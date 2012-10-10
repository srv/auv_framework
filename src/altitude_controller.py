#!/usr/bin/python

PACKAGE = 'auv_control'

import roslib; roslib.load_manifest(PACKAGE)
import rospy
import dynamic_reconfigure.server
from altitude_controller.cfg import AltitudeControllerConfig
from pid import Pid
from sensor_msgs.msg import Range
from geometry_msgs.msg import Wrench

class AltitudeControllerNode():
    """
    Node for calculating the wrench needed in order to keep the altitude to the 
    seabed constant. As only one DOF -namely z- is controlled, the
    other values have to be given by some other input, e.g. via joystick.

    This node subscribes to 'altitude', expecting a sensor_msgs/Range message
    and to 'wrench_input' which should be of type geometry_msgs/Wrench.
    The node also receives std_msgs/Float32 messages on the topic 
    'altitude_request' to define the required altitude (setpoint).
    The node then calculates the force in z needed to control the AUV to
    have the desired altitude, replaces this value within wrench_input and
    publishes the result to 'wrench_output'.

    Internally a simple PID is used for control, its variables can be modified
    using dynamic_reconfigure.
    """
    def __init__(self, frequency, min_force, max_force, k_p, k_i, k_d):
        self.wrench_input = Wrench()
        self.started = False;
        self.pid = pid(k_p, k_i, k_d, min_force, max_force)
        self.server = dynamic_reconfigure.server.Server(AltitudeControllerConfig, self.reconfigure)
        
        self.pub = rospy.Publisher('wrench_output', Wrench)
        
        rospy.Subscriber('wrench_input', WrenchStamped, self.wrenchInputCallback)
        rospy.Subscriber('altitude_request', Float32, self.setpointCallback)
        
        period = rospy.rostime.Duration.from_sec(1.0/frequency)
        self.timer = rospy.Timer(period, self.updateOutput)
        
        rospy.Subscriber('altitude', Float32, self.altitudeCallback)
        
    def reconfigure(self, config, level):
        rospy.loginfo("Reconfigure request...")
        self.pid.setConstants(config['Kp'], config['Ki'], config['Kd'])
        rospy.loginfo("Reconfigured!")
        return config # Returns the updated configuration.
    
    def setpointCallback(self,setpoint):
        """
        Change the setPoint of the controller
        """
        rospy.loginfo('Changed setpoint to: %s', setpoint.data)
        self.pid.setSetpoint(setpoint.data)

    def altitudeCallback(self, altitude):
        self.current_altitude = altitude.data
        if not self.started:
            self.started = True
        
    def wrenchInputCallback(self, wrench):
        self.wrench_input = wrench
        
    def updateOutput(self, event):
        if self.started:
            wrench_output = self.wrench_input
            dt = (event.current_real - event.last_real).toSec()
            wrench_output.force.z = self.pid.update(self.current_altitude, dt)
            self.pub.publish(wrench_output)
        else:
            rospy.loginfo('Waiting topics to be published...')

if __name__ == "__main__":
    rospy.init_node('altitude_controller')
    try:
        frequency = rospy.get_param("~frequency", 10.0)
        min_force = rospy.get_param("~min_force", -100.0);
        max_force = rospy.get_param("~max_force",  100.0);
        k_p = rospy.get_param("~Kp",  0.0);
        k_i = rospy.get_param("~Ki",  0.0);
        k_d = rospy.get_param("~Kd",  0.0);
        node = AltitudeControllerNode(
                frequency, min_force, max_force, k_p, k_i, k_d)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
