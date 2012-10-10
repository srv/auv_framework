#!/usr/bin/python

PACKAGE = 'auv_control'

import roslib; roslib.load_manifest(PACKAGE)
import rospy
import sys
import argparse
import dynamic_reconfigure.server
from altitude_controller.cfg import AltitudeControllerConfig
from pid import PID
from std_msgs.msg import Float32
from srv_msgs.msg import Depth
from geometry_msgs.msg import WrenchStamped

class AltitudeControllerNode():
    """
    Node for calculating the thrust needed in the motors
    in order to keep the altitude to the seabed constant
    Publishes wrench topic to the motors while subscribes
    to joystick motor levels and altitude or depth topics.
    """

    def __init__(self, input_subscriber, frequency):
        kp = 0
        ki = 0
        kd = 0
        derivator = 0
        integrator = 0
        antiwindup_max = 100;
        antiwindup_min = -100;
        self.user_wrench_levels = WrenchStamped()
        self.started = False;
        self.PID = PID(kp,ki,kd,derivator,integrator,antiwindup_max,antiwindup_min)
        self.server = dynamic_reconfigure.server.Server(AltitudeControllerConfig, self.reconfigure)
        
        self.pub = rospy.Publisher('motor_wrench_levels', WrenchStamped)
        
        rospy.Subscriber('joy_wrench_levels', WrenchStamped, self.joyCallback)
        rospy.Subscriber('controller_setpoint', Float32, self.setpointCallback)
        
        period = rospy.rostime.Duration.from_sec(1/frequency)
        self.timer = rospy.Timer(period, self.updateOutput)
        
        assert input_subscriber == 'depth' or input_subscriber == 'altitude', "input can only take as value altitude or depth: %r" % input
        if input_subscriber == 'depth':
            rospy.Subscriber('depth', Depth, self.depthCallback)
        elif input_subscriber == 'altitude':
            rospy.Subscriber('altitude', Float32, self.altitudeCallback)
        
        
        
    def reconfigure(self, config, level):
        rospy.loginfo("Reconfigure request...")
        
        self.PID.setKp(config['Kp'])
        config['Kp'] = self.PID.Kp
        
        self.PID.setKi(config['Ki'])
        config['Ki'] = self.PID.Ki
        
        self.PID.setKd(config['Kd'])
        config['Kd'] = self.PID.Kd 
        
        self.PID.setIntegrator(config['Integrator'])
        config['Integrator'] = self.PID.getIntegrator()
        
        self.PID.setDerivator(config['Derivator'])
        config['Derivator'] = self.PID.getDerivator()
      
        rospy.loginfo("Reconfigured!")
        return config # Returns the updated configuration.
    
    def setpointCallback(self,setpoint):
        """
        Change the setPoint of the controller
        """
        rospy.loginfo('Changed setpoint to: %s', setpoint.data)
        self.PID.setPoint(setpoint.data)

    def altitudeCallback(self, altitude):
        """
        Update the output with the new feedback
        self.error = self.set_point - current_value
        """
        self.currentValue = altitude.data
        if not self.started:
            self.started = True
        
    def depthCallback(self, depth):
        """
        Update the output with the new feedback
        self.error = self.set_point - current_value
        """
        self.currentValue= depth.data
        if not self.started:
            self.started = True
        
    def joyCallback(self, joy_wrench_levels):
        self.user_wrench_levels = joy_wrench_levels;
        
    def updateOutput(self, event):
        if self.started:
            self.output = self.PID.update(self.currentValue)
            motor_wrench_levels = self.user_wrench_levels
            if self.output > 1.0:
                self.output = 1.0
            elif self.output < -1.0:
                self.output = -1.0
            motor_wrench_levels.wrench.force.z = self.output
            self.pub.publish(motor_wrench_levels)
        else:
            rospy.loginfo('Waiting topics to be published...')

if __name__ == "__main__":
    rospy.init_node('altitude_controller')
    parser = argparse.ArgumentParser(description='PID controller for the altitude of a submarine')
    parser.add_argument('-i','--input', default='altitude', help='input to controller (altitude or depth)')
    parser.add_argument('-f','--frequency',type=float, default=5.0, help='controller frequency in hertz')
    args = parser.parse_args(rospy.myargv()[1:])    
    try:
        node = AltitudeControllerNode(args.input,args.frequency)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
