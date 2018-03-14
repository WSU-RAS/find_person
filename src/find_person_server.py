#!/usr/bin/env python

# importing the libraries
import roslib
# roslib.manifest('FindPerson')
import rospy
import actionlib

import find_person.msg

# libraries pertaining to the pan tilt servos
import sys
import argparse
from copy import copy
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, \
	FollowJointTrajectoryGoal

# libraries pertaining to query person location from database
import json
from object_detection_msgs.srv import ObjectQuery, ObjectQueryResponse
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
import datetime
import dateutil.parser

# class for pan tilt the servos
class Trajectory(object):
    def __init__(self):
        ns = 'head_controller/'
        self._client = actionlib.SimpleActionClient(
            ns + "follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )
        self._goal = FollowJointTrajectoryGoal()
        server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))
        if not server_up:
            rospy.logerr("Timed out waiting for Joint Trajectory"
                         " Action Server to connect. Start arbotix node.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)
        self.clear()
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


# findperson action server
class FindPersonServer(object):

	# to query position of human
	def getObjectLocation(self, name):
		try:
			query = rospy.ServiceProxy("query_object", ObjectQuery)
			result = query(name)
			return result.locations
		except rospy.ServiceException, e:
			rospy.roserr("Service call failed: %s" % e)
		return None


	# messages that are used to publish results and feedback
	_goal = find_person.msg.FindPersonGoal()
	_feedback = find_person.msg.FindPersonFeedback()
	_result = find_person.msg.FindPersonResult()

	def __init__ (self, name):

		self._action_name = name
		# instantiating the object of the trajectory class
		self.traj = Trajectory()
		
		# register all the callbacks
		self._as = actionlib.SimpleActionServer(self._action_name, find_person.msg.FindPersonAction, execute_cb =  self.execute_cb, auto_start =  False)
		
		# start the server
		self._as.start()
		rospy.loginfo("Server succesfully started..")

	# callback function for deciding when to call the action client for the rotation servos
	def execute_cb(self, goal):
		r = rospy.Rate(1)
		success = True
		
		data = self.getObjectLocation('human')
		# data = [12, 4]
		# if the time stamp  location of the person is not updated in the range of 10 seconds
		# if (True):
		if  (datetime.datetime.now - dateutil.parser.parse(data.time)) > datetime.timedelta(seconds = 10):
			rospy.loginfo("Timestamp of detected human expired!")
			rospy.loginfo("Initiating servo rotation..")
			rospy.loginfo(goal)
			
			if goal.task_number == (1.0 or 4.0):
					# here the servos are rotated to left
					self.traj.add_point ([0.5, 1.0], 5.0)
					self.traj.start()
					self.traj.wait(15.0)
					success = True
			elif goal.task_number == (2.0 or 3.0 or 5.0):
					# servos are rotated to right
					self.traj.add_point([-0.5, 0.0], 10.0)
					self.traj.start()
					self.traj.wait(15.0)
					success =  True
		if success:
			self._result.found = True
			# self._result.location = [12., 4.]
			if len(locations) != 0:
				self._result.location = [data.locations[0].x, data.locations[0].y]
			self._as.set_succeeded(self._result)


if __name__ == '__main__':
	rospy.init_node('find_person_server')
	server = FindPersonServer(rospy.get_name())
	rospy.spin()
