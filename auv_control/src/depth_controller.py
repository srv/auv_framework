#!/usr/bin/python

PACKAGE = 'auv_control'

import roslib; roslib.load_manifest(PACKAGE)
import rospy
import dynamic_reconfigure.server
import copy
import auv_control_msgs.srv
from auv_control.cfg import DepthControllerConfig
from pid import Pid
from auv_sensor_msgs.msg import Depth
from std_msgs.msg import Float32
from geometry_msgs.msg import WrenchStamped

class DepthControllerNode():
    """
    Node for calculating the wrench needed in order to keep the depth  
    constant. As only one DOF -namely z- is controlled, the
    other values have to be given by some other input, e.g. via joystick.

    This node subscribes to 'depth', expecting a auv_sensor_msgs/Depth message
    and to 'wrench_input' which should be of type geometry_msgs/WrenchStamped.
    The node also receives std_msgs/Float32 messages on the topic 
    'depth_request' to define the required depth (setpoint).
    The node then calculates the force in z needed to control the AUV to
    have the desired depth, replaces this value within wrench_input and
    publishes the result to 'wrench_output'.

    Internally a simple PID is used for control, its variables can be modified
    using dynamic_reconfigure.

    A service is provided to enable and disable the depth control, by default
    it is disabled. Enabling sets the setpoint to the current depth.
    """
    def __init__(self, frequency):
        self.FEEDBACK_TIMEOUT = 1.0 # half a second
        self.wrench_input = WrenchStamped()
        self.setpoint_valid = False
        self.feedback_received = False
        self.enabled = False
        self.last_feedback = Depth()
        self.enable_server = rospy.Service('~enable', auv_control_msgs.srv.EnableControl, self.enable)
        self.pid = Pid(0.0, 0.0, 0.0) # the right constants will
        self.k_f = 0.0                # be set through dynamic reconfigure
        self.server = dynamic_reconfigure.server.Server(DepthControllerConfig, self.reconfigure)
        
        self.pub = rospy.Publisher('~wrench_output', WrenchStamped)
        
        rospy.Subscriber('~wrench_input', WrenchStamped, self.wrenchInputCallback)
        rospy.Subscriber('~depth_request', Float32, self.setpointCallback)
        
        period = rospy.rostime.Duration.from_sec(1.0/frequency)
        self.timer = rospy.Timer(period, self.updateOutput)
        
        rospy.Subscriber('depth', Depth, self.depthCallback)
        rospy.loginfo('Listening for depth feedback to be published on '
                      '%s...', rospy.resolve_name('depth'))
        rospy.loginfo('Waiting for setpoint to be published on '
                      '%s...', rospy.resolve_name('~depth_request'))

    def enable(self, request):
        """
        Handles ROS service requests for enabling/disabling control.
        Returns current enabled status and setpoint.
        """
        response = auv_control_msgs.srv.EnableControlResponse()
        if request.enable:
            if self.isFeedbackValid():
                self.enabled = True
                self.pid.setSetpoint(self.last_feedback.depth)
                self.setpoint_valid = True
                response.enabled = True
            else:
                rospy.logerr("Cannot enable depth control without valid feedback!")
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
        self.k_f = config['Kf']
        rospy.loginfo("Reconfigured to (Kp, Ki, Kd, Kf) = (%f, %f, %f, %f)", 
                self.pid.k_p, self.pid.k_i, self.pid.k_d, self.k_f)
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

    def depthCallback(self, depth_msg):
        self.last_feedback = depth_msg
        
    def wrenchInputCallback(self, wrench):
        self.wrench_input = wrench

    def isFeedbackValid(self):
        time_since_last_feedback = (rospy.Time.now() - self.last_feedback.header.stamp).to_sec()
        feedback_in_time = time_since_last_feedback < self.FEEDBACK_TIMEOUT
        return feedback_in_time and (self.last_feedback.depth > 0)
        
    def updateOutput(self, event):
       if self.setpoint_valid and self.enabled:
           wrench_output = copy.deepcopy(self.wrench_input)
           if self.isFeedbackValid():
               dt = (event.current_real - event.last_real).to_sec()
               wrench_output.wrench.force.z = self.pid.update(self.last_feedback.depth, dt) + self.k_f
               if wrench_output.wrench.force.z > 1.0:
                   wrench_output.wrench.force.z = 1.0
               if wrench_output.wrench.force.z < -1.0:
                   wrench_output.wrench.force.z = -1.0
           else:
               rospy.logwarn("Depth feedback is invalid, setting force in z to zero.")
               wrench_output.wrench.force.z = 0.0
           wrench_output.header.stamp = rospy.Time.now()
           self.pub.publish(wrench_output)

if __name__ == "__main__":
    rospy.init_node('depth_controller')
    try:
        frequency = rospy.get_param("~frequency", 10.0)
        rospy.loginfo('Starting depth control with %f Hz.\n'
                      '      depth: %s \n'
                      '  wrench_input: %s \n'
                      ' wrench_output: %s \n', frequency, 
                      rospy.resolve_name('depth'), 
                      rospy.resolve_name('~wrench_input'),
                      rospy.resolve_name('~wrench_output'))

        node = DepthControllerNode(frequency)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
