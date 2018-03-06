#! /usr/bin/env/ python

# importing the libraries
import roslib
roslib.manifest('FindPerson')
import rospy
import actionlib

import find_person.msg

class FindPersonServer(object):
	_goal = findperson.msg.FindPersonGoal()
	_feedback = findperson.msg.FindPersonFeedback()
	_result = findperson.msg.FindPersonResult()

	def __init__ (self, name):
		self._action_name = name
		
		# register all the callbacks
		self._as = actionlib.SimpleActionServer(self._action_name, findperson.msg.FindPersonAction, execute_cb =  self.goal_cb, autostart =  False)
		
		# here we subscribe to the bounding boxes of the detected humans
		# callback used is the analysis_cb
		self.subscribe = rospy.Subscriber('/human_detection', self.analysis_cb, queue_size = 1)
		
		# start the server
		self._as.start()
	# callback function for goal
	def goal_cb():
		rospy.loginfo("Receiving error at step and sending instructons to turn"{})
		self.goal = as_.acceptNewGoal(error_point)


	# callback function for deciding when to call the action client for the otation servos
	def analysis_cb(self, data):
		if (!as_.isActive()):
			return
		
		while (self.subscribe is True):

			if data.pick is False:
				# basically if the bounding box is not found then using the task number we will send the 
				# required goal to the servos for panning it to tilt
				if taskNumber is 1, 4:
					# send goal to pan tilt to the left
					# here we call the action client 
					# TODO: How to call the action client

				elif taskNumber is 2, 3, 5:
					# send goal to pan tilt to the right


if __name__ == 'main':
	rospy.init_node('find_person_server')
	server = FindPersonServer()
	rospy.spin()


