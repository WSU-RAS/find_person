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

INITIAL_PAN = 0.0
INITIAL_TILT = 0.25

# class for pan tilt the servos
class PanTiltController(object):
    def __init__(self):
        # set up client to work with the pan/tilt head
        ns = 'head_controller/'
        self._client = actionlib.SimpleActionClient(
            ns + "follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )

        # goal
        self._goal = FollowJointTrajectoryGoal()

        # set up client to server
        server_up = self._client.wait_for_server(timeout=rospy.Duration(60.0))
        if not server_up:
            rospy.logerr("Timed out waiting for Joint Trajectory"
                         " Action Server to connect. Start arbotix node.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)
        self.clear()

        # internal location references
        self.current_pan = 0.0
        self.current_tilt = 0.0

        # set the pan_tilt head to its initial location


    # goto commands
    def goto_neutral(self):
        """ move to the neutral position
        """
        self.goto(INITIAL_PAN, INITIAL_TILT)

    def zero(self):
        """ move to zero position

        move both the pan and tilt to their respective zero positions

        """
        self.goto(0.0, 0.0)

    def goto(self, pan, tilt):
        """ goto a given location

        move the pan and tilt to the locations specified by pan and tilt

        """
        self.add_point([pan, tilt], 2.0)
        self.start()
        self.wait(2.0)
        self.clear()

    # relative commands
    def pan_left(self, amount):
        """ pan left by the given amount

        move the pan servo to the left by the given amount.
        TODO figure out the conversion rate between the float and the degrees and change amount to degrees

        """
        self.goto(self.current_pan + amount, self.current_tilt)

    def pan_right(self, amount):
        """ pan left by the given amount

        move the pan servo to the left by the given amount.
        TODO figure out the conversion rate between the float and the degrees and change amount to degrees

        """
        self.goto(self.current_pan - amount, self.current_tilt)

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
