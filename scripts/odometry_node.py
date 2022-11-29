#!/usr/bin/env python3

import rospy
import math
import numpy as np
from ias0220_201400.msg import counter_message, Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler


WHEEL_RADIUS        = 0.035
WHEEL_SEPARATION    = 0.165
COUNTS_OF_ROTATION  = 1440
MAX_ROTATION_DIFF   = COUNTS_OF_ROTATION / 2


class OdometryNode():

    def __init__(self):

        self.time_diff = None
        self.prev_time = None
        self.prev_time_diff = None
        self.prev_encoder_output = None

        self.odomMsg = Odometry()
        self.odomMsg.header.frame_id = "odom"
        self.odomMsg.child_frame_id = "base_link"
        self.odomMsg.header.stamp = rospy.Time.now()

        self.odomPose = self.odomMsg.pose.pose
        self.odomTwist = self.odomMsg.twist.twist

        self.odomPose.position.x = 0.
        self.odomPose.position.y = 0.
        self.odomPose.position.z = 0.

        self.odomPose.orientation.x = 0.
        self.odomPose.orientation.y = 0.
        self.odomPose.orientation.z = 0.
        self.odomPose.orientation.w = 0.

        self.odomTwist.linear.x = 0.
        self.odomTwist.linear.y = 0.
        self.odomTwist.linear.z = 0.

        self.odomTwist.angular.x = 0.
        self.odomTwist.angular.y = 0.
        self.odomTwist.angular.z = 0.

        self.pub = rospy.Publisher("/odom", Odometry, queue_size=10)   
        self.sub = rospy.Subscriber("/encoders_output", counter_message, self.odom_cb)


    def odom_cb(self, msg):
        
        curr_time = rospy.get_time()
        curr_encoder_output = msg

        if self.prev_time is not None and self.prev_encoder_output is not None:

            curr_time_diff = curr_time - self.prev_time
            time_diff = curr_time_diff if curr_time_diff > 0 else self.prev_time_diff
            self.prev_time_diff = time_diff

            # Calculate encoder turns based on revolution count
            left_encoder_val = self.calc_encoder_turns(curr_encoder_output.count_left, self.prev_encoder_output.count_left)
            right_encoder_val = self.calc_encoder_turns(curr_encoder_output.count_right, self.prev_encoder_output.count_right)

            # Calculate wheel velocities
            left_wheel_vel = self.calc_wheel_vel(left_encoder_val, time_diff)
            right_wheel_vel = self.calc_wheel_vel(right_encoder_val, time_diff)

            # Calculate robot velocities and pose
            angular_vel, linear_vel = self.calc_robot_vel(left_wheel_vel, right_wheel_vel)
            robot_pose = self.calc_robot_pose(angular_vel, linear_vel, time_diff)

            robot_pose_quat = quaternion_from_euler(0., 0., robot_pose[2][0])

            # Assign robot velocities and pose to appropriate Odometry message attributes.
            self.odomTwist.linear.x = linear_vel
            self.odomTwist.angular.z = angular_vel

            self.odomPose.position.x = robot_pose[0][0]
            self.odomPose.position.y = robot_pose[1][0]

            self.odomPose.orientation.x = robot_pose_quat[0]
            self.odomPose.orientation.y = robot_pose_quat[1]
            self.odomPose.orientation.z = robot_pose_quat[2]
            self.odomPose.orientation.w = robot_pose_quat[3]

            # Publish to /odom
            self.pub.publish(self.odomMsg) 
        
        self.prev_time = curr_time
        self.odomMsg.header.stamp = rospy.Time.from_sec(curr_time)
        self.prev_encoder_output = curr_encoder_output


    def calc_robot_pose(self, angular_vel, linear_vel, time_diff):
        """
        Given the robot's velocities and time diferrence, this function calculates the robot's pose

        """
        # Get robot's displacement by integration velocities
        robot_frame = np.array([ [linear_vel], [0], [angular_vel] ])
        robot_displacement = robot_frame * time_diff

        # Build previous world frame for pose calculation
        prev_robot_pose_quat = [self.odomPose.orientation.x, self.odomPose.orientation.y, self.odomPose.orientation.z, self.odomPose.orientation.w]
        prev_robot_pose_yaw = euler_from_quaternion(prev_robot_pose_quat)[2]
        prev_robot_pose = np.array([ [self.odomPose.position.x], [self.odomPose.position.y], [prev_robot_pose_yaw] ])

        # Build rotation matrix
        theta = prev_robot_pose_yaw + (robot_displacement[2][0] / 2.)
        c, s = math.cos(theta), math.sin(theta)
        rotation_matrix = np.array([ [c, -s, 0], [s, c, 0], [0, 0, 1] ])

        # Calculate current pose using the rotation matrix, robot frame displacement and previous robot pose
        robot_pose = rotation_matrix @ robot_displacement + prev_robot_pose
        return robot_pose


    def calc_robot_vel(self, left_wheel_vel, right_wheel_vel):
        """
        Given the wheel velocities, this function calculates the robot's angular and linear velocity

        """
        angular_vel = (right_wheel_vel - left_wheel_vel) / WHEEL_SEPARATION
        linear_vel = (right_wheel_vel + left_wheel_vel) / 2.
        return angular_vel, linear_vel


    def calc_wheel_vel(self, encoder_val, time_diff):
        """
        Given an encoder value and a time difference, this function calculates the robot wheel's linear velocity

        """
        return (encoder_val / COUNTS_OF_ROTATION) * ((2 * math.pi * WHEEL_RADIUS) / time_diff) 


    def calc_encoder_turns(self, curr_encoder_output, prev_encoder_output):
        """
        This function calculates the difference between the current and previous encoder output.
        It takes into consideration the fact that encoder value is between a fixed range

        """
        turns = curr_encoder_output - prev_encoder_output

        # Robot moving forward and completed rotation
        if turns < 0 and abs(turns) > MAX_ROTATION_DIFF:
            turns = (COUNTS_OF_ROTATION + curr_encoder_output) - prev_encoder_output

        # Robot moving backward and completed rotation
        elif turns > 0 and abs(turns) > MAX_ROTATION_DIFF:
            turns = curr_encoder_output - (COUNTS_OF_ROTATION + prev_encoder_output)

        return turns


    @staticmethod
    def run():
        """
        This function keeps the node alive.

        """
        rospy.spin()



if __name__ == '__main__':
    try:
        rospy.init_node('odometry_node', log_level=rospy.DEBUG)
        odom_node = OdometryNode()
        odom_node.run()
    except rospy.ROSInterruptException:
        pass
