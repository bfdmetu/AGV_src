#!/usr/bin/env python

""" nav_test.py - Version 1.1 2013-12-20

    Command a robot to move autonomously among a number of goal locations defined in the map frame.
    On each round, select a new random sequence of locations, then attempt to move to each location
    in succession.  Keep track of success rate, time elapsed, and total distance traveled.

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:

    http://www.gnu.org/licenses/gpl.html

"""

import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from random import sample
from math import pow, sqrt

import tf2_ros
from geometry_msgs.msg import PoseStamped
import sys

class NavTest():
    def __init__(self):
        rospy.init_node('nav_test', anonymous=True)

        buf = tf2_ros.Buffer()
        tf_l = tf2_ros.TransformListener(buf)

        map_frame_id = rospy.get_param('~map_frame_id', 'map')
        base_frame_id = rospy.get_param('~base_frame_id', 'base_link')
        topic_name = rospy.get_param('~topic_name', 'initialpose')


        rospy.on_shutdown(self.shutdown)

        # How long in seconds should the robot pause at each location?
        self.rest_time = rospy.get_param("~rest_time", 10)

        # Are we running in the fake simulator?
        self.fake_test = rospy.get_param("~fake_test", False)

        # Goal state return values
        goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED',
                       'SUCCEEDED', 'ABORTED', 'REJECTED',
                       'PREEMPTING', 'RECALLING', 'RECALLED',
                       'LOST']

        # Set up the goal locations. Poses are defined in the map frame.
        # An easy way to find the pose coordinates is to point-and-click
        # Nav Goals in RViz when running in the simulator.
        # Pose coordinates are then displayed in the terminal
        # that was used to launch RViz.
        locations = dict()
        initial_points = dict()


        # Simulation
        # locations['point1'] = Pose(Point(1.7676, -2.4355, 0.000), Quaternion(0.000, 0.000, -0.5789, 0.8153))
        # locations['point2'] = Pose(Point(1.58585, -3.64425, 0.000), Quaternion(0.000, 0.000, 0.97219, -0.23416))
        # locations['point3'] = Pose(Point(0.0396, -4.41539, 0.000), Quaternion(0.000, 0.000,-0.70223, 0.711947))
        # locations['point4'] = Pose(Point(0.15242, -5.47837, 0.000), Quaternion(0.000, 0.000, -0.6898, 0.72397))
        # locations['point5'] = Pose(Point(0.0927, -7.1939, 0.000), Quaternion(0.000, 0.000, -0.68932, 0.72445))

        # initial_points['point1'] = Pose(Point(-0.07291, -0.25183, 0.000), Quaternion(0.000, 0.000,  0.038105, 0.99927))
        # initial_points['point2'] = Pose(Point(0.15242, -5.47837, 0.000), Quaternion(0.000, 0.000, -0.6898, 0.72397))
        # initial_points['point3'] = Pose(Point(1.58585, -3.64425, 0.000), Quaternion(0.000, 0.000, 0.97219, -0.23416))
        # initial_points['point4'] = Pose(Point(0.0396, -4.41539, 0.000), Quaternion(0.000, 0.000,-0.70223, 0.711947))

        # Short
        # locations['point1'] = Pose(Point(-2.499, -0.019, 0.000), Quaternion(0.000, 0.000, 0.999, -0.012))
        # locations['point2'] = Pose(Point(-3.4, -0.025, 0.000), Quaternion(0.000, 0.000, 0.999, -0.012))
        # locations['point3'] = Pose(Point(-3.6, -0.025, 0.000), Quaternion(0.000, 0.000, 0.999, -0.012))
        # Long
        locations['point1'] = Pose(Point(-7.570, -0.133, 0.000), Quaternion(0.000, 0.000, 0.999, -0.004))
        locations['point2'] = Pose(Point(-19.988, -0.486, 0.000), Quaternion(0.000, 0.000, 0.999, 0.011))
        locations['point3'] = Pose(Point(-34.465, -0.664, 0.000), Quaternion(0.000, 0.000, 0.999, -0.012))
        locations['point4'] = Pose(Point(-49.815, -1.147, 0.000), Quaternion(0.000, 0.000, 0.997, 0.074))
        locations['point5'] = Pose(Point(-61.034, 0.430, 0.000), Quaternion(0.000, 0.000, 0.991, 0.137))
        locations['point6'] = Pose(Point(-63.797, 4.495, 0.000), Quaternion(0.000, 0.000, 0.723, 0.690))
        locations['point7'] = Pose(Point(-61.940, 8.052, 0.000), Quaternion(0.000, 0.000, 0.359, 0.933))
        locations['point8'] = Pose(Point(-49.569, 8.927, 0.000), Quaternion(0.000, 0.000, -0.006, 0.999))
        locations['point9'] = Pose(Point(-40.283, 9.028, 0.000), Quaternion(0.000, 0.000, -0.006, 0.999))
        locations['point10'] = Pose(Point(-21.218, 9.423, 0.000), Quaternion(0.000, 0.000, 0.015, 0.999))
        locations['point11'] = Pose(Point(-9.431, 9.842, 0.000), Quaternion(0.000, 0.000, -0.007, 0.999))
        locations['point12'] = Pose(Point(0.678, 9.954, 0.000), Quaternion(0.000, 0.000, 0.017, 0.999))
        initial_points['point1'] = Pose(Point(-0.07291, -0.25183, 0.000), Quaternion(0.000, 0.000, 0.999, -0.012))
        initial_points['point3'] = Pose(Point(-34.465, -0.664, 0.000), Quaternion(0.000, 0.000, 0.999, -0.012))


        # Publisher to manually control the robot (e.g. to stop it, queue_size=5)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)

        # set initialpose
        rospy.loginfo("start test inital pose...")
        initialpose_pub = rospy.Publisher("initialpose",PoseWithCovarianceStamped,latch=True, queue_size=1)

        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        rospy.loginfo("Waiting for move_base action server...")

        # Wait 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))

        rospy.loginfo("Connected to move base server")

        # A variable to hold the initial pose of the robot to be set by
        # the user in RViz
        initial_pose = PoseWithCovarianceStamped()

        # Variables to keep track of success rate, running time,
        # and distance traveled
        n_locations = len(locations)
        n_goals = 0
        n_successes = 0
        # 4 9 11
        i = 0
        j = 0
        distance_traveled = 0
        start_time = rospy.Time.now()
        running_time = 0
        location = ""
        last_location = ""


        # p = PoseWithCovarianceStamped()
        # p.header.stamp = rospy.Time.now()
        # p.header.frame_id = "map"
        # p.pose.pose = initial_points["point1"]
        # p.pose.covariance[6 * 0 + 0] = 0.5 * 0.5
        # p.pose.covariance[6 * 1 + 1] = 0.5 * 0.5
        # p.pose.covariance[6 * 3 + 3] = 3.14159 / 12.0 * 3.14159 / 12.0
        # initialpose_pub.publish(p)

        # # Get the initial pose from the user
        # rospy.loginfo("*** Click the 2D Pose Estimate button in RViz to set the robot's initial pose...")
        # rospy.wait_for_message('initialpose', PoseWithCovarianceStamped)
        # self.last_location = Pose()
        # rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.update_initial_pose)

        # Make sure we have the initial pose
        while initial_pose.header.stamp == "":
            rospy.sleep(3)

        rospy.loginfo("Starting navigation test")

        # Begin the main loop and run through a sequence of locations
        while not rospy.is_shutdown():
            # If we've gone through the current sequence,break
            if i == n_locations + 1:
                continue
            if i == n_locations :
                i += 1
                rospy.loginfo("*** Finish the current works!!! ...")
                continue

            # Get the next location in the current sequence
            location = "point"+str(i+1)

            #unable the initialpose
            if (i+1) == 0 :
                initial_point = "point"+str(i)

                p = PoseWithCovarianceStamped()
                p.header.stamp = rospy.Time.now()
                p.header.frame_id = "map"
                p.pose.pose = initial_points[initial_point]
                p.pose.covariance[6 * 0 + 0] = 0.5 * 0.5
                p.pose.covariance[6 * 1 + 1] = 0.5 * 0.5
                p.pose.covariance[6 * 3 + 3] = 3.14159 / 12.0 * 3.14159 / 12.0
                initialpose_pub.publish(p)

            # Keep track of the distance traveled.
            # Use updated initial pose if available.
            if initial_pose.header.stamp == "":
                distance = sqrt(pow(locations[location].position.x -
                                    locations[last_location].position.x, 2) +
                                pow(locations[location].position.y -
                                    locations[last_location].position.y, 2))
            else:
                rospy.loginfo("Updating current pose.")
                distance = sqrt(pow(locations[location].position.x -
                                    initial_pose.pose.pose.position.x, 2) +
                                pow(locations[location].position.y -
                                    initial_pose.pose.pose.position.y, 2))
                initial_pose.header.stamp = ""

            # Store the last location for distance calculations
            last_location = location

            # Increment the counters
            i += 1
            n_goals += 1

            # Set up the next goal location
            self.goal = MoveBaseGoal()
            self.goal.target_pose.pose = locations[location]
            self.goal.target_pose.header.frame_id = 'map'
            self.goal.target_pose.header.stamp = rospy.Time.now()

            # Let the user know where the robot is going next
            rospy.loginfo("Going to: " + str(location))

            # Start the robot toward the next location
            self.move_base.send_goal(self.goal)

            # Allow 5 minutes to get there
            finished_within_time = self.move_base.wait_for_result(rospy.Duration(300))

            # Check for success or failure
            if not finished_within_time:
                self.move_base.cancel_goal()
                rospy.loginfo("Timed out achieving goal")
            else:
                state = self.move_base.get_state()
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("Goal succeeded!")
                    n_successes += 1
                    distance_traveled += distance
                    rospy.loginfo("State:" + str(state))
                else:
                  rospy.loginfo("Goal failed with error code: " + str(goal_states[state]))

            # How long have we been running?
            running_time = rospy.Time.now() - start_time
            running_time = running_time.secs / 60.0

            # Print a summary success/failure, distance traveled and time elapsed
            rospy.loginfo("Success so far: " + str(n_successes) + "/" +
                          str(n_goals) + " = " +
                          str(100 * n_successes/n_goals) + "%")
            rospy.loginfo("Running time: " + str(trunc(running_time, 1)) +
                          " min Distance: " + str(trunc(distance_traveled, 1)) + " m")
            # rospy.sleep(self.rest_time)

    def update_initial_pose(self, initial_pose):
        self.initial_pose = initial_pose

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_goal()
        # rospy.sleep(2)
        self.cmd_vel_pub.publish(Twist())
        # rospy.sleep(1)

def trunc(f, n):
    # Truncates/pads a float f to n decimal places without rounding
    slen = len('%.*f' % (n, f))
    return float(str(f)[:slen])

if __name__ == '__main__':
    try:
        NavTest()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("AMCL navigation test finished.")
