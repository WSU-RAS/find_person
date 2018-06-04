#!/usr/bin/env python

# import common packages
import sys
import argparse
from copy import copy

# import library packages
import actionlib
import rospy
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, \
                             FollowJointTrajectoryGoal

# class for pan tilt the servos
class PanTiltController(object):
    def __init__(self):
        # set up client to work with the pan/tilt head
        ns = 'head_controller/'
        self._client = actionlib.SimpleActionClient(
            ns + "follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )
        self._goal = FollowJointTrajectoryGoal()
        server_up = self._client.wait_for_server(timeout=rospy.Duration(60.0))
        if not server_up:
            rospy.logerr("Timed out waiting for Joint Trajectory"
                         " Action Server to connect. Start arbotix node.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)
        self.clear()

    # higher level commands
    def pan_left(self):
        self.add_point([0.0, 0.25], 0.0)
        self.add_point([2.0, 0.25], 10.0)
        self.add_point([0.0, 0.25], 11.0)
        self.start()
        self.wait(11.0)
        return True

    def pan_right(self):
        self.add_point([0.0, 0.25], 0.0)
        self.add_point([-2.0, 0.25], 10.0)
        self.add_point([0.0, 0.25], 11.0)
        self.start()
        self.wait(11.0)
        return True

    # lower level commands
    def add_point(self, positions, time):
        point = JointTrajectoryPoint()
        point.positions = copy(positions)
        point.time_from_start = rospy.Duration(time)
        self._goal.trajectory.points.append(point)

    def start(self):
        self._goal.trajectory.header.stamp = rospy.Time.now()
        self._client.send_goal(self._goal)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=15.0):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))

    def result(self):
        return self._client.get_result()

    def clear(self):
        self._goal = FollowJointTrajectoryGoal()
        self._goal.trajectory.joint_names = ["head_" + joint + "_joint" for joint in \
                ["pan", "tilt"]]
