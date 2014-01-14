#!/usr/bin/python

PACKAGE = 'auv_control'

import roslib; roslib.load_manifest(PACKAGE)
import rospy
import dynamic_reconfigure.server
import auv_control_msgs.srv
from auv_control.cfg import TwistControllerConfig
from pid import Pid
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

class TwistControllerNode():
    """
    Node for controlling the twist (linear and angular speed) of a robot,
    using a simple PID scheme. The input consists in a setpoint (required speed)
    given as geometry_msgs/TwistStamped on topic 'twist_request' and of
    current odometry readings as nav_msgs/Odometry on topic 'odometry'.
    Every degree of freedom has a separate PID.
    The velocity values given in the incoming odometry message are used as
    feedback for the PIDs.
    Output is a message of type geometry_msgs/WrenchStamped on topic 
    'wrench' containing force and torque values that are
    necessary to obtain the requested speed.
    """
    def __init__(self, frequency):
        self.FEEDBACK_TIMEOUT = 1.0
        self.setpoint_valid = False
        self.feedback_received = False
        self.enabled = False
        self.last_feedback = Odometry()
        self.enable_server = rospy.Service('~enable', auv_control_msgs.srv.EnableControl, self.enable)
        self.pids = []
        for i in range(6):
            self.pids.append(Pid(0.0, 0.0, 0.0))
        self.server = dynamic_reconfigure.server.Server(TwistControllerConfig, self.reconfigure)
        
        self.pub = rospy.Publisher('~wrench_output', WrenchStamped)
        rospy.Subscriber('~twist_request', TwistStamped, self.setpointCallback)
        
        period = rospy.rostime.Duration.from_sec(1.0/frequency)
        self.timer = rospy.Timer(period, self.updateOutput)
        
        rospy.Subscriber('odometry', Odometry, self.odometryCallback)
        rospy.loginfo('Listening for altitude feedback to be published on '
                      '%s...', rospy.resolve_name('altitude'))
        rospy.loginfo('Waiting for setpoint to be published on '
                      '%s...', rospy.resolve_name('~twist_request'))

    def enable(self, request):
        """
        Handles ROS service requests for enabling/disabling control.
        Returns current enabled status and setpoint.
        """
        response = auv_control_msgs.srv.EnableControlResponse()
        if request.enable:
            if self.isFeedbackValid():
                self.enabled = True
                self.setpoint_valid = True
                response.enabled = True
            else:
                rospy.logerr("Cannot enable twist control without valid feedback!")
                response.enabled = False
        else:
            self.enabled = False
            response.enabled = False
        return response

    def reconfigure(self, config, level):
        """
        Handles dynamic reconfigure requests.
        """
        rospy.loginfo("Reconfigure request...")
        self.pids[0].k_p = config['linear_x_Kp']
        self.pids[0].k_i = config['linear_x_Ki']
        self.pids[0].k_d = config['linear_x_Kd']
        self.pids[1].k_p = config['linear_y_Kp']
        self.pids[1].k_i = config['linear_y_Ki']
        self.pids[1].k_d = config['linear_y_Kd']
        self.pids[2].k_p = config['linear_z_Kp']
        self.pids[2].k_i = config['linear_z_Ki']
        self.pids[2].k_d = config['linear_z_Kd']
        self.pids[3].k_p = config['angular_x_Kp']
        self.pids[3].k_i = config['angular_x_Ki']
        self.pids[3].k_d = config['angular_x_Kd']
        self.pids[4].k_p = config['angular_y_Kp']
        self.pids[4].k_i = config['angular_y_Ki']
        self.pids[4].k_d = config['angular_y_Kd']
        self.pids[5].k_p = config['angular_z_Kp']
        self.pids[5].k_i = config['angular_z_Ki']
        self.pids[5].k_d = config['angular_z_Kd']

#        rospy.loginfo("Reconfigured to (Kp, Ki, Kd, Kf) = (%f, %f, %f, %f)", 
#                self.pid.k_p, self.pid.k_i, self.pid.k_d, self.k_f)
        return config # Returns the updated configuration.
    
    def setpointCallback(self,setpoint):
        """
        Change the setpoint of the controller.
        """
        if not self.setpoint_valid:
            rospy.loginfo("First setpoint received.")
            self.setpoint_valid = True
        setSetpoint(setpoint.twist.twist)
        rospy.loginfo('Changed setpoint to: %s', setpoint.twist.twist)

    def setSetpoint(self, twist):
        self.pids[0].setSetpoint(twist.linear.x)
        self.pids[1].setSetpoint(twist.linear.y)
        self.pids[2].setSetpoint(twist.linear.z)
        self.pids[3].setSetpoint(twist.angular.x)
        self.pids[4].setSetpoint(twist.angular.y)
        self.pids[5].setSetpoint(twist.angular.z)

    def odometryCallback(self, odometry_msg):
        self.last_feedback = odometry_msg
        
    def isFeedbackValid(self):
        feedback_in_time = (rospy.Time.now() - 
            self.last_feedback.header.stamp).to_sec() < self.FEEDBACK_TIMEOUT
        return feedback_in_time 
        
    def updateOutput(self, event):
       if self.setpoint_valid and self.enabled:
           wrench_output = WrenchStamped()
           if self.isFeedbackValid():
               dt = (event.current_real - event.last_real).to_sec()
               wrench_output.wrench.force.x = self.pids[0].update(self.last_feedback.twist.twist.linear.x, dt)
               wrench_output.wrench.force.y = self.pids[1].update(self.last_feedback.twist.twist.linear.y, dt)
               wrench_output.wrench.force.z = self.pids[2].update(self.last_feedback.twist.twist.linear.z, dt)
               wrench_output.wrench.torque.x = self.pids[3].update(self.last_feedback.twist.twist.angular.x, dt)
               wrench_output.wrench.torque.y = self.pids[4].update(self.last_feedback.twist.twist.angular.y, dt)
               wrench_output.wrench.torque.z = self.pids[5].update(self.last_feedback.twist.twist.angular.z, dt)
           else:
               rospy.logwarn("Odometry feedback is invalid, setting wrench to zero.")
           wrench_output.header.stamp = rospy.Time.now()
           self.pub.publish(wrench_output)

if __name__ == "__main__":
    rospy.init_node('twist_controller')
    try:
        frequency = rospy.get_param("~frequency", 10.0)
        rospy.loginfo('Starting twist control with %f Hz.\n', frequency)
        node = TwistControllerNode(frequency)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

