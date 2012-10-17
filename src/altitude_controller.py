#!/usr/bin/python

PACKAGE = 'auv_control'

import roslib; roslib.load_manifest(PACKAGE)
import rospy
import dynamic_reconfigure.server
import copy
import auv_control.srv
from auv_control.cfg import AltitudeControllerConfig
from pid import Pid
from sensor_msgs.msg import Range
from std_msgs.msg import Float32
from geometry_msgs.msg import WrenchStamped

class AltitudeControllerNode():
    """
    Node for calculating the wrench needed in order to keep the altitude to the 
    seabed constant. As only one DOF -namely z- is controlled, the
    other values have to be given by some other input, e.g. via joystick.

    This node subscribes to 'altitude', expecting a sensor_msgs/Range message
    and to 'wrench_input' which should be of type geometry_msgs/WrenchStamped.
    The node also receives std_msgs/Float32 messages on the topic 
    'altitude_request' to define the required altitude (setpoint).
    The node then calculates the force in z needed to control the AUV to
    have the desired altitude, replaces this value within wrench_input and
    publishes the result to 'wrench_output'.

    Internally a simple PID is used for control, its variables can be modified
    using dynamic_reconfigure.

    A service is provided to enable and disable the altitude control, by default
    it is disabled. Enabling sets the setpoint to the current altitude.
    """
    def __init__(self, frequency, k_p, k_i, k_d):
        self.FEEDBACK_TIMEOUT = 0.5 # half a second
        self.wrench_input = WrenchStamped()
        self.setpoint_valid = False
        self.feedback_received = False
        self.enabled = False
        self.last_feedback = Range()
        self.enable_server = rospy.Service('~enable', auv_control.srv.EnableControl, self.enable)
        self.pid = Pid(k_p, k_i, k_d)
        self.server = dynamic_reconfigure.server.Server(AltitudeControllerConfig, self.reconfigure)
        
        self.pub = rospy.Publisher('~wrench_output', WrenchStamped)
        
        rospy.Subscriber('~wrench_input', WrenchStamped, self.wrenchInputCallback)
        rospy.Subscriber('~altitude_request', Float32, self.setpointCallback)
        
        period = rospy.rostime.Duration.from_sec(1.0/frequency)
        self.timer = rospy.Timer(period, self.updateOutput)
        
        rospy.Subscriber('altitude', Range, self.altitudeCallback)
        rospy.loginfo('Listening for altitude feedback to be published on '
                      '%s...', rospy.resolve_name('altitude'))
        rospy.loginfo('Waiting for setpoint to be published on '
                      '%s...', rospy.resolve_name('~altitude_request'))

    def enable(self, request):
        """
        Handles ROS service requests for enabling/disabling control.
        Returns current enabled status and setpoint.
        """
        response = auv_control.srv.EnableControlResponse()
        if request.enable:
            if self.isFeedbackValid():
                self.enabled = True
                self.pid.setSetpoint(self.last_feedback.range)
                self.setpoint_valid = True
                response.enabled = True
            else:
                rospy.logerr("Cannot enable altitude control without valid feedback!")
                response.enabled = False
        else:
            self.enabled = False
            response.enabled = False
        response.current_setpoint = self.pid.getSetpoint()
        return response

    def reconfigure(self, config, level):
        """
        Handles dynamic reconfigure requests.
        """
        rospy.loginfo("Reconfigure request...")
        self.pid.k_p = config['Kp']
        self.pid.k_i = config['Ki']
        self.pid.k_d = config['Kd']
        rospy.loginfo("Reconfigured to (Kp, Ki, Kd) = (%f, %f, %f)", 
                self.pid.k_p, self.pid.k_i, self.pid.k_d)
        return config # Returns the updated configuration.
    
    def setpointCallback(self,setpoint):
        """
        Change the setpoint of the controller.
        """
        if not self.setpoint_valid:
            rospy.loginfo("First setpoint received.")
            self.setpoint_valid = True
        rospy.loginfo('Changed setpoint to: %s', setpoint.data)
        self.pid.setSetpoint(setpoint.data)

    def altitudeCallback(self, range_msg):
        self.last_feedback = range_msg
        
    def wrenchInputCallback(self, wrench):
        self.wrench_input = wrench

    def isFeedbackValid(self):
        feedback_in_time = (rospy.Time.now() - 
            self.last_feedback.header.stamp).to_sec() < self.FEEDBACK_TIMEOUT
        return feedback_in_time and (self.last_feedback.range > 0)
        
    def updateOutput(self, event):
       if self.setpoint_valid and self.enabled:
           wrench_output = copy.deepcopy(self.wrench_input)
           if self.isFeedbackValid():
               dt = (event.current_real - event.last_real).to_sec()
               wrench_output.wrench.force.z = -self.pid.update(self.last_feedback.range, dt)
               if wrench_output.wrench.force.z > 1.0:
                   wrench_output.wrench.force.z = 1.0
               if wrench_output.wrench.force.z < -1.0:
                   wrench_output.wrench.force.z = -1.0
           else:
               rospy.logwarn("Altitude feedback is invalid, setting force in z to zero.")
               wrench_output.wrench.force.z = 0.0
           wrench_output.header.stamp = rospy.Time.now()
           self.pub.publish(wrench_output)

if __name__ == "__main__":
    rospy.init_node('altitude_controller')
    try:
        frequency = rospy.get_param("~frequency", 10.0)
        k_p = rospy.get_param("~Kp",  0.0);
        k_i = rospy.get_param("~Ki",  0.0);
        k_d = rospy.get_param("~Kd",  0.0);
        rospy.loginfo('Starting altitude control with the following settings: \n'
                      '     frequency: %f \n'
                      '            Kp: %f \n'
                      '            Ki: %f \n'
                      '            Kd: %f \n'
                      '      altitude: %s \n'
                      '  wrench_input: %s \n'
                      ' wrench_output: %s \n', frequency, k_p, k_i, k_d,
                      rospy.resolve_name('altitude'), 
                      rospy.resolve_name('~wrench_input'),
                      rospy.resolve_name('~wrench_output'))

        node = AltitudeControllerNode(
                frequency, k_p, k_i, k_d)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
