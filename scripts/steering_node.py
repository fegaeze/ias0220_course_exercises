#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion


class Steering():

    def __init__(self):

        self.twist = Twist()
        self.can_move = True

        self.twist.linear.x = 0.
        self.twist.linear.y = 0.
        self.twist.linear.z = 0.

        self.twist.angular.x = 0.
        self.twist.angular.y = 0.
        self.twist.angular.z = 0.

        self.imu_sub = rospy.Subscriber("/imu", Imu, self.steer_cb)
        self.distance_sub = rospy.Subscriber("/distance", Int16, self.distance_cb)

        self.robot_pub = rospy.Publisher("dr_diffdrive_controller/cmd_vel", Twist, queue_size=10)  
        self.rate = rospy.Rate(10)		


    def steer_cb(self, msg):
        
        msg_quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        robot_rpy = euler_from_quaternion(msg_quat)

        if self.can_move is False:
            self.twist.linear.x = 0.
            self.twist.angular.z = 0.
        else:
            self.twist.linear.x = robot_rpy[0]
            self.twist.angular.z = robot_rpy[2]

        self.robot_pub.publish(self.twist)
        self.rate.sleep() 

    
    def distance_cb(self, msg):
        if(msg.data <= 10):
            self.can_move = False
        else:
            self.can_move = True


    @staticmethod
    def run():
        """
        This function keeps the node alive.

        """
        rospy.spin()


if __name__ == '__main__':
    try:
        rospy.init_node('steering')
        steer_node = Steering()
        steer_node.run()
    except rospy.ROSInterruptException:
        pass
