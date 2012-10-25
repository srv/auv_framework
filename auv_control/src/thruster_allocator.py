#!/usr/bin/python

PACKAGE = 'auv_control'
import roslib; roslib.load_manifest(PACKAGE)
import rospy
from geometry_msgs.msg import WrenchStamped
from auv_control_msgs.msg import MotorLevels
import numpy

class ThrusterAllocationNode():
    """
    Node for calculating the thruster speed needed to apply a given wrench
    to an AUV. The underlying theory is an affine thruster model as described
    in 'Guidance and Control of Ocean Vehicles' by T. Fossen, section 4.1, 
    pp. 96-97. The calculation is done using a thruster allocation matrix (TAM)
    which defines how requested forces in each degree of freedom have to be
    translated to motor speeds.

    This node subscribes to 'wrench', expecting a geometry_msgs/WrenchStamped.
    Whenever a new message arrives on this topic, the desired motor speeds
    are calculated and published on the topic 'motor_levels' as
    auv_control_msgs/MotorLevels.
    """
    def __init__(self, tam):
        self.tam = tam
        rospy.Subscriber('~wrench', WrenchStamped, self.wrenchCallback)
        self.pub = rospy.Publisher('~motor_levels', MotorLevels)
        rospy.loginfo('Listening for wrench requests on '
                      '%s...', rospy.resolve_name('~wrench'))
        rospy.loginfo('Publishing motor levels on '
                      '%s...', rospy.resolve_name('~motor_levels'))
        self.signed_sqrt_v = numpy.vectorize(self.signed_sqrt)

    def signed_sqrt(self, number):
        return -numpy.sqrt(-number) if number < 0 else numpy.sqrt(number)

    def wrenchCallback(self, wrench_stamped_msg):
        wrench = numpy.array([
                wrench_stamped_msg.wrench.force.x,
                wrench_stamped_msg.wrench.force.y,
                wrench_stamped_msg.wrench.force.z,
                wrench_stamped_msg.wrench.torque.x,
                wrench_stamped_msg.wrench.torque.y,
                wrench_stamped_msg.wrench.torque.z])
        motor_levels = self.signed_sqrt_v(numpy.dot(self.tam, wrench))
        motor_levels_msg = MotorLevels()
        motor_levels_msg.header.stamp = wrench_stamped_msg.header.stamp
        motor_levels_msg.header.frame_id = 'base_link'
        motor_levels_msg.levels = motor_levels.tolist()
        self.pub.publish(motor_levels_msg)

if __name__ == "__main__":
    rospy.init_node('thruster_allocator')
    taminfo = rospy.get_param("~thruster_allocation_matrix")
    tam = numpy.array(taminfo['data']).reshape(taminfo['rows'], taminfo['cols'])
    node = ThrusterAllocationNode(tam)
    rospy.spin()
