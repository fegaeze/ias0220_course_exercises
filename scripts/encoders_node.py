#!/usr/bin/env python3
"""
publishes encoder output data based on a fake absolute encoder for the two wheels of a differentially steered robot

@author: Simon Pierre Godon
@contact: simon.godon@ttu.ee
@creation_date: 25-08-2020
"""

import roslib
import rospy
import math
import tf
from ias0220_201400.msg import counter_message


class EncodersNode():
    """
    uses /tf to publish the count of the encoder
    publishes topic "encoders_output"
    """

    def __init__(self):
        self.cpr = 1440.0 							# clicks per revolution
        self.listener = tf.TransformListener()
        self.count_publisher = rospy.Publisher(
            "/encoders_output", counter_message, queue_size=1000)
        self.rate = rospy.Rate(25)					# rate of spinning: 25Hz

    def corrector(self, x, y):
        """
        Corrects the instability of the transformation from quaternion to Euler angles.
        """
        if x < 1:
            if y > 0:
                return y
            else:
                return y+360
        if x > 1:
            return 180-y

    def get_count(self, rot):
        """ Takes a quaternion as an input and outputs the absolute encoder count of the rotation between the two frames. """
        quaternion = (rot[0], rot[1], rot[2], rot[3])
        euler = tf.transformations.euler_from_quaternion(quaternion)
        roll = euler[0]*180.0/math.pi                # converting rad to degree
        pitch = euler[1]*180.0/math.pi
        interval = 360.0/self.cpr
        # correcting the instability of the conversion quaternion to euler
        count = self.corrector(roll, pitch)/interval
        return int(math.modf(count)[1])

    def step(self):
        """
Performs one iteration of class loop.
Publishes click count for each wheel based on the transformations between base_link and the wheels.
        """
        try:
            rot_left = self.listener.lookupTransform(
                'base_link', 'left_wheel', rospy.Time(0))[1]
            rot_right = self.listener.lookupTransform(
                'base_link', 'right_wheel', rospy.Time(0))[1]
        except (tf.LookupException, tf.ConnectivityException):
            return
        self.count_publisher.publish(self.get_count(
            rot_left), self.get_count(rot_right))

    def run(self):
        """ Main loop of class. steps node at set intervals, if data from sensors is available."""
        while not rospy.is_shutdown():
            self.step()
            self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node('encoders_node')
    encoders = EncodersNode()
    encoders.run()
