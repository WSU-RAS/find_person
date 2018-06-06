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
    """ Class for controlling the pan_tilt servos on the RAS bot

    Note that right is negative and left is positive for the pan servo.

    Attributes:
        initial_pan - constant, initial pan to start the pan_tilt with on init
        initial_tilt - constant, initial tilt to start the pan_tilt with on init
        max_left - constant, maximum left the pan servo can go to in double values
        max_right - constant, maximum right the tilt servo can go to in double values
        max_up - constant, maximum up the pan servo can go to in double values
        max_down - constant, maximum down the pan servo can go to in double values

    """
    initial_pan = 0.0
    initial_tilt = 0.25
    max_left = 2.0
    max_right = -2.0
    max_up = 0.5
    max_down = 0.0

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
        self.goto_neutral()

    # goto commands
    def goto_neutral(self):
        """ move to the neutral position
        """
        self.goto(PanTiltController.initial_pan, PanTiltController.initial_tilt)

    def zero(self):
        """ move to zero position

        move both the pan and tilt to their respective zero positions

        """
        self.goto(0.0, 0.0)

    def goto(self, pan, tilt):
        """ goto a given location

        move the pan and tilt to the locations specified by pan and tilt.
        Update the internal state to reflect we have moved.

        """
        # move to location
        self.add_point([pan, tilt], 2.0)
        self.start()
        self.wait(2.0)
        self.clear()

        # update our internal location
        self.current_pan = pan
        self.current_tilt = tilt

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
