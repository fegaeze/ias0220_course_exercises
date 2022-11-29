#!/usr/bin/env python3

"""
Templete for home assignment 7 (Robot Control).
Node to take a set of waypoints and to drive a differential
drive robot through those waypoints using a simple PD controller
and provided odometry data.

Students should complete the code. Note the places marked with "# TODO".

@author: ias0220 teachers
@date: September 2020
@input: Odometry as nav_msgs Odometry message
@output: body velocity commands as geometry_msgs Twist message;
         list of waypoints as MarkerArray message
"""

import tf
import math
import rospy
from geometry_msgs.msg import Twist, Point, PoseStamped
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
#  [rad] if error in heading greater than this value, turn on the spot
angle_threshold = 0.01


class PDController:
    def __init__(self):

        """Get the static parameters from the parameter server (which have
         been loaded onto the server from the yaml file using the launch file).
         The namespace defined in the launch file must match the namespace
          used here (i.e., "controller_waypoints")"""
        self.Kd = rospy.get_param("/controller_waypoints/controller/Kd")
        self.Kp = rospy.get_param("/controller_waypoints/controller/Kp")
        self.distance_margin = rospy.get_param("/controller_waypoints/mission/distance_margin")
        self.waypoints = rospy.get_param("/controller_waypoints/mission/waypoints")

        # Print the parameters
        rospy.loginfo("Got static data from parameter server:")
        rospy.loginfo(" Mission parameters:")
        rospy.loginfo("  Distance margin: %.2f", self.distance_margin)
        rospy.loginfo("  Waypoints (#: x|y):")
        wp = 1
        for waypoint in self.waypoints:
            rospy.loginfo("   %d: %.1f|%.1f", wp, waypoint[0], waypoint[1])
            wp += 1
        rospy.loginfo(" Controller parameters:")
        rospy.loginfo("  Proportional gains: %.2f, %.2f",
                      self.Kp[0],
                      self.Kp[1])
        rospy.loginfo("  Derivative gains  : %.2f, %.2f",
                      self.Kd[0],
                      self.Kd[1])
        rospy.loginfo("-----------------------------------------------------")

        # Initialization of class variables
        self.wpIndex = 0          # counts the visited waypoints
        self.position = Point()   # current position (3D vector: .x, .y, .z)
        self.heading = 0.0        # current orientation of robot (rad yaw)
        self.done = False
        self.init = True
        self.vel_cmd = [0.0, 0.0]  # calculated velocities (linear, angular)

        self.prev_linear_error = 0.0
        self.prev_angular_error = 0.0

        self.rate = rospy.Rate(10.0)
        self.listener = tf.TransformListener()

        # Publishers and subscribers
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.onNavGoal)
        rospy.Subscriber('/dr_diffdrive_controller/odom', Odometry, self.onOdom)
        self.publisher_cmd_vel = rospy.Publisher("/controller_diffdrive/cmd_vel",Twist, queue_size=10)
        self.publisher_waypoints = rospy.Publisher("/mission_control/waypoints", MarkerArray, queue_size=10)

        # Messages
        self.twist = None
        self.marker_array = None

    # Registering start time of this node
        self.startTime = 0
        while self.startTime == 0:
            self.startTime = rospy.Time.now().to_sec()

    def wrapAngle(self, angle):
        """
        Helper function that returns angle wrapped between +- Pi.
        Hint: Pass your error in heading [rad] into this function, and it
        returns the shorter angle. This prevents your robot from turning
        along the wider angle and makes it turn along the smaller angle (but
        in opposite direction) instead.
        @param: self
        @param: angle - angle to be wrapped in [rad]
        @result: returns wrapped angle -Pi <= angle <= Pi
        """
        if angle > math.pi:
            return self.wrapAngle(angle - 2*math.pi)
        elif angle < -math.pi:
            return self.wrapAngle(angle + 2*math.pi)
        else:
            return angle

    def rad2deg(self, angle):
        """
        Helper function to transform an angle from radians to degrees.
        @param: self
        @param: angle - angle in [rad]
        @result: returns angle in [deg]
        """
        angle = 180.0 * angle / math.pi
        return angle

    def run(self):
        """
        Main loop of class.
        Since we are using a callback function (onOdom) to trigger our
        computations and outputs, we don't need a main loop here. But then we
        must ensure the node does not terminate.
        @param: self
        @result: runs the step function for controller update
        """
        # spin() simply keeps python from exiting until this node is stopped
        rospy.loginfo("Waiting for odom message...")
        rospy.spin()

    def setNextWaypoint(self):
        """
        Removes current waypoint from list and sets next one as current target.
        @param: self
        @result: returns True if the next waypoint exists and has been set,
                 otherwise False
        """
        if not self.waypoints:
            return False
        self.waypoints.pop(0)
        if not self.waypoints:
            return False
        self.wpIndex += 1
        rospy.loginfo("----------------------------------------------")
        rospy.loginfo("                Next waypoint                 ")
        rospy.loginfo("----------------------------------------------")
        return True
    
    def isWaypointReached(self):
        """
        Checks if waypoint is reached based on pre-defined threshold.
        @param: self
        @result: returns True if waypoint is reached, otherwise False
        """

        if not self.waypoints:
            return False
        
        desired_point = self.waypoints[0]
        current_point = self.position
        distance = math.sqrt((current_point[0] - desired_point[0])**2 + (current_point[1] - desired_point[1])**2)

        if distance < self.distance_margin:
            return True

        return False

    def controller(self):
        """
        Takes the errors and calculates velocities from it, according to
         control algorithm specs.
        @param: self
        @result: sets the values in self.vel_cmd
        """
        # Output 0 (skip all calculations) if the last waypoint was reached
        if self.done:
            self.vel_cmd = [0.0, 0.0]
            return

        if not self.waypoints:
            return False

        current_point = self.position
        desired_point = self.waypoints[0]

        current_orientation = self.heading
        desired_orientation = math.atan2((desired_point[1] - current_point[1]), (desired_point[0] - current_point[0]))

        linear_error = math.sqrt((current_point[0] - desired_point[0])**2 + (current_point[1] - desired_point[1])**2)
        angular_error = self.wrapAngle(desired_orientation - current_orientation)

        time_passed = 1 / 50

        linear_derivative = (linear_error - self.prev_linear_error) / time_passed
        angular_derivative = (angular_error - self.prev_angular_error) / time_passed

        pd_linear = (self.Kp[0]*linear_error) + (self.Kd[0]*linear_derivative)
        pd_angular = (self.Kp[1]*angular_error) + (self.Kd[1]*angular_derivative)

        self.prev_linear_error = linear_error
        self.prev_angular_error = angular_error

        if angular_error > angle_threshold:
            self.vel_cmd = [0.0, pd_angular]
        else:
            self.vel_cmd = [pd_linear, pd_angular]

    def publish_vel_cmd(self):
        """
        Publishes command velocities computed by the control algorithm.
        @param: self
        @result: publish message
        """
        self.twist = Twist()
        self.twist.linear.x = self.vel_cmd[0]
        self.twist.angular.z = self.vel_cmd[1]

        self.publisher_cmd_vel.publish(self.twist)

    def publish_waypoints(self):
        """
        Publishes the list of waypoints, so RViz can see them.
        @param: self
        @result: publish message
        """
        self.marker_array = MarkerArray()
        id = 0
        for waypoint in self.waypoints:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = waypoint[0]
            marker.pose.position.y = waypoint[1]
            marker.pose.position.z = 0.05
            marker.id = id
            id += 1
            self.marker_array.markers.append(marker)
        self.publisher_waypoints.publish(self.marker_array)

    def onOdom(self, odom_msg):
        """
        Callback function, handling incoming odometry messages.
        @param: self
        @param odom_msg - odometry geometry message
        @result: update of relevant vehicle state variables
        """

        try:
            (translation, quat_rotation) = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return
        
        euler_rotation = euler_from_quaternion(quat_rotation)

        # Store odometry data
        self.position = translation
        self.heading = euler_rotation[2]

        # Check if current target reached;
        #  set next one if necessary and possible
        if self.isWaypointReached():
            if not self.setNextWaypoint():
                if not self.done:
                    rospy.loginfo("This was the last waypoint in the list.")
                    endTime = rospy.Time.now().to_sec()
                    rospy.loginfo("Started node  [s]: %.2f", self.startTime)
                    rospy.loginfo("Finished node [s]: %.2f", endTime)
                    totalTime = endTime - self.startTime
                    rospy.loginfo("Elapsed time  [s]: %.2f", totalTime)
                    self.done = True

        # Apply PD algorithm
        self.controller()

        # Publish velocity commands
        self.publish_vel_cmd()

        # Publish waypoint list
        self.publish_waypoints()

    def onNavGoal(self, goal_msg):
        self.waypoints.append([goal_msg.pose.position.x, goal_msg.pose.position.y])


if __name__ == '__main__':
    rospy.init_node("Planner")
    controller = PDController()
    controller.run()
