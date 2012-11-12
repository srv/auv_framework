#!/usr/bin/env python
import roslib; roslib.load_manifest('auv_control')

import string
import roslib.message
import rospy
import rostopic

from std_srvs.srv import Empty as EmptyService

from geometry_msgs.msg import WrenchStamped
from auv_control_msgs.msg import MotorLevels

class Error(Exception):
    def __init__(self, error):
        self.error = error
    def __str__(self):
        return repr(self.error)

class ParsingError(Exception):
    pass

class SafetyError(Exception):
    pass

class FrequencyError(SafetyError):
    pass

class FieldLimitError(SafetyError):
    def __init__(self, field, limits, value):
        err = field + " limits exceeded: " + str(limits) + " value=" + str(value)
        super(FieldLimitError, self).__init__(err)

class Limit(object):
    """
    Class to describe a limit for a value. Put None as min or max
    if they have not to be taken into account.
    """
    def __init__(self, min_value, max_value):
        self.minimum = min_value
        self.maximum = max_value
    def is_ok(self, value):
        if self.minimum:
            if value < self.minimum:
                return False
        if self.maximum:
            if value > self.maximum:
                return False
        return True
    def __str__(self):
        return "min=" + repr(self.minimum) + " max=" + repr(self.maximum)

class FieldMonitor(object):
    """
    Class to monitor a field inside a message.
    """
    def __init__(self, field_name, value_limit):
        self.field_eval = self.generate_field_eval(field_name)
        self.field_name = field_name
        self.value_limit = value_limit

    @classmethod
    def create(cls, params):
        min_value = params['min_value'] if 'min_value' in params else None
        max_value = params['max_value'] if 'max_value' in params else None
        return cls(params['field'], Limit(min_value, max_value))
        
    def check(self, msg):
        value = self.field_eval(msg)
        if not self.value_limit.is_ok(value):
            raise FieldLimitError(self.field_name, self.value_limit, value)

    @staticmethod
    def _get_field_attr_recursive(obj, name):
        if '.' in name:
            fields = name.rsplit('.', 1)
            return FieldMonitor._get_field_attr(FieldMonitor._get_field_attr_recursive(obj, fields[0]), fields[1])
        else:
            return FieldMonitor._get_field_attr(obj, name)

    @staticmethod
    def _get_field_attr(obj, name):
        try:
            if '[' in name:
                field, rest = name.split('[')
                slot_num = string.atoi(rest[:rest.find(']')])
                return getattr(obj, field).__getitem__(slot_num)
            else:
                return getattr(obj, name)
        except Exception as e:
            raise ParsingError("cannot parse field reference [%s]: %s"%(name, str(e)))

    @staticmethod
    def generate_field_eval(field):
        """
        :param field: message field name (e.g. "wrench.force.x") ``str``
        :returns: function to evaluate the given field on a message:
                  fn(msg)->msg.field
        """
        def fn(msg):
            fields = '.'.join(filter(None, field.split('/')))
            return FieldMonitor._get_field_attr_recursive(msg, fields)
        return fn
 
class TopicMonitor(object):
    """
    Subscriber to ROS topic that supervises the topic's frequency and values
    inside the incoming messages. The frequency is checked every second in
    _timer_callback(), the message field values are checked on each incoming
    message.
    When an error is found (frequency or field values outside limits),
    an error flag is set.
    The frequency is calculated as a mean over 10 messages.
    """
    def __init__(self, topic, frequency_limit, field_monitors):
        self.SUBSCRIBING_TIMEOUT = 5.0
        self.error = None
        self.sub = None
        self.WINDOW_SIZE = 10
        self.topic = topic
        self.frequency_limit = frequency_limit
        self.field_monitors = field_monitors
        self.start_time = rospy.Time.now()
        self.message_times = []
        timer_duration = rospy.rostime.Duration.from_sec(1.0)
        self.topic_timer = rospy.Timer(timer_duration, self._check_subscribe_topic)

    @classmethod
    def create(cls, params):
        """
        Factory method to create a topic monitor with given parameters
        :param params: parameters, such as
        {topic: '/depth_sensor/depth', min_frequency: 1, field_monitors: {...}}
        """
        field_monitors = []
        for field_monitor_params in params['field_monitors']:
            field_monitors.append(FieldMonitor.create(field_monitor_params))
        min_frequency = \
                params['min_frequency'] if 'min_frequency' in params else None
        max_frequency = \
                params['max_frequency'] if 'max_frequency' in params else None
        return cls(
                params['topic'], 
                Limit(min_frequency, max_frequency), field_monitors)
       
    def _message_callback(self, msg):
        """
        ROS subscriber callback
        :param msg: ROS message data
        """
        # store current time for frequency checking
        self.message_times.append(rospy.Time.now())
        if len(self.message_times) > self.WINDOW_SIZE:
            self.message_times.pop(0)
        try:
            for field_monitor in self.field_monitors:
                field_monitor.check(msg)
        except FieldLimitError as e:
            self.error = e

    def _check_subscribe_topic(self, event):
        if not self.sub:
            topic_type, real_topic, _ = rostopic.get_topic_type(self.topic, blocking=False)
            if not topic_type and \
                    (rospy.Time.now() - self.start_time).to_sec() > self.SUBSCRIBING_TIMEOUT:
                self.error = "Monitored topic " + self.topic + " not available!"
            elif topic_type:
                data_class = roslib.message.get_message_class(topic_type)
                if data_class:
                    self.sub = rospy.Subscriber(real_topic, data_class, self._message_callback)
                    timer_duration = rospy.rostime.Duration.from_sec(1.0)
                    self.frequency_timer = rospy.Timer(timer_duration, self._check_frequency)

    def _check_frequency(self, event):
        if self.message_times:
            duration = (rospy.Time.now() - self.message_times[0]).to_sec()
            messages_received = len(self.message_times)
        else:
            duration = (rospy.Time.now() - self.start_time).to_sec()
            messages_received = 0
        frequency = messages_received / duration if duration > 0 else 0
        # check for the minimum if present and only if enough time has passed to
        # be able to check it
        if self.frequency_limit.minimum:
            if duration > 1.0 / self.frequency_limit.minimum:
                if frequency < self.frequency_limit.minimum:
                    self.error = FrequencyError("Frequency of " + self.topic + " is too low!")
        if self.frequency_limit.maximum:
            if frequency > self.frequency_limit.maximum:
                self.error = FrequencyError("Frequency of " + self.topic + " is too high!")

class SafetyControllerNode(object):
    """
    Node that subscribes to wrench and motor levels inputs, while monitoring
    topic frequencies and values. If everything is ok, the inputs are forwarded.
    If an error is detected, i.e. an error flag of a topic monitor is set,
    no more inputs are forwarded and the vehicle is brought to the surface
    by sending a negative z wrench.
    """
    def __init__(self, frequency, topic_monitors):
        self.topic_monitors = topic_monitors
        self.wrench_input = WrenchStamped()
        self.INPUT_TIMEOUT = 5.0 
        self.last_input_time = rospy.Time.now()
        self.safety_error = False
        rospy.Subscriber('wrench_request', WrenchStamped, self.wrenchCallback)
        rospy.Subscriber('motor_levels_request', MotorLevels, self.motorLevelsCallback)
        self.wrench_pub = rospy.Publisher('wrench', WrenchStamped)
        self.motor_levels_pub = rospy.Publisher('motor_levels', MotorLevels)
        period = rospy.rostime.Duration.from_sec(1.0/frequency)
        self.timer = rospy.Timer(period, self.checkForErrors)
        self.reset_service = rospy.Service('~reset', EmptyService, self.reset)
 
    def wrenchCallback(self, wrench):
        if not self.safety_error:
            self.wrench_pub.publish(wrench)
        self.last_input_time = rospy.Time.now()

    def motorLevelsCallback(self, levels):
        if not self.safety_error:
            self.motor_levels_pub.publish(levels)
        self.last_input_time = rospy.Time.now()

    def checkForErrors(self, event):
        """
        Checks for two kinds of errors safety error and input timeout.
        If the safety monitors detect an error, a wrench will be sent with
        all values set to zero but force.z set to -1 to bring the vehicle up.
        If the last input is longer ago that self.INPUT_TIMEOUT, a zero
        wrench is published to stop the vehicle.
        """
        for topic_monitor in self.topic_monitors:
            if topic_monitor.error:
                rospy.logerr("Safety Error: " + str(topic_monitor.error))
                self.safety_error = True
        input_in_time = (rospy.Time.now() - 
                self.last_input_time).to_sec() < self.INPUT_TIMEOUT
        if self.safety_error or not input_in_time:
            wrench_msg = WrenchStamped()
            wrench_msg.header.frame_id = 'base_link'
            wrench_msg.header.stamp = rospy.Time.now()
            if self.safety_error:
                wrench_msg.wrench.force.z = -1
            self.wrench_pub.publish(wrench_msg)

    def reset(self, req):
        for topic_monitor in self.topic_monitors:
            topic_monitor.error = None
        self.safety_error = False
        wrench_msg = WrenchStamped()
        wrench_msg.header.frame_id = 'base_link'
        wrench_msg.header.stamp = rospy.Time.now()
        self.wrench_pub.publish(wrench_msg)

if __name__ == "__main__":
    rospy.init_node('auv_safety_manager')
    frequency = rospy.get_param("~frequency", 1.0) # error checking frequency
    try:
        topic_monitor_params_list = rospy.get_param("~topic_monitors")
        topic_monitors = []
        for topic_monitor_params in topic_monitor_params_list:
            if topic_monitor_params:
                topic_monitors.append(TopicMonitor.create(topic_monitor_params))
        controller_node = SafetyControllerNode(frequency, topic_monitors)
        rospy.spin()
    except KeyError as e:
        rospy.logerr("Error getting parameters: " + repr(e))
    except ParsingError as e:
        rospy.logerr("Error parsing parameters: " + repr(e))
    
