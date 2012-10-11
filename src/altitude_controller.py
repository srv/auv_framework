#!/usr/bin/python

PACKAGE = 'auv_control'

import roslib; roslib.load_manifest(PACKAGE)
import rospy
import dynamic_reconfigure.server
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
    """
    def __init__(self, frequency, k_p, k_i, k_d):
        self.wrench_input = WrenchStamped()
        self.started = False;
        self.pid = Pid(k_p, k_i, k_d)
        self.server = dynamic_reconfigure.server.Server(AltitudeControllerConfig, self.reconfigure)
        
        self.pub = rospy.Publisher('wrench_output', WrenchStamped)
        
        rospy.Subscriber('wrench_input', WrenchStamped, self.wrenchInputCallback)
        rospy.Subscriber('altitude_request', Float32, self.setpointCallback)
        
        period = rospy.rostime.Duration.from_sec(1.0/frequency)
        self.timer = rospy.Timer(period, self.updateOutput)
        
        rospy.Subscriber('altitude', Range, self.altitudeCallback)
        
    def reconfigure(self, config, level):
        rospy.loginfo("Reconfigure request...")
        self.pid.k_p = config['Kp']
        self.pid.k_i = config['Ki']
        self.pid.k_d = config['Kd']
        rospy.loginfo("Reconfigured to (Kp, Ki, Kd) = (%f, %f, %f)", 
                self.pid.k_p, self.pid.k_i, self.pid.k_d)
        return config # Returns the updated configuration.
    
    def setpointCallback(self,setpoint):
        """
        Change the setPoint of the controller
        """
        rospy.loginfo('Changed setpoint to: %s', setpoint.data)
        self.pid.setSetpoint(setpoint.data)

    def altitudeCallback(self, range_msg):
        self.current_altitude = range_msg.range
        if not self.started:
            self.started = True
        
    def wrenchInputCallback(self, wrench):
        self.wrench_input = wrench
        
    def updateOutput(self, event):
        if self.started:
            wrench_output = self.wrench_input
            dt = (event.current_real - event.last_real).toSec()
            wrench_output.wrench.force.z = self.pid.update(self.current_altitude, dt)
            self.pub.publish(wrench_output)
        else:
            rospy.loginfo('Waiting for altitude feedback to be published on '
                          '%s...', rospy.resolve_name('altitude'))

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
                      rospy.resolve_name('wrench_input'),
                      rospy.resolve_name('wrench_output'))

        node = AltitudeControllerNode(
                frequency, k_p, k_i, k_d)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
