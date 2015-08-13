#!/usr/bin/env python

import socket
import time
import random
import sys
import rospy
import actionlib
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction

from twisted.internet import reactor
from twisted.internet.protocol import Factory, Protocol


class Turtling(Protocol):
	def __init__(self, ip, port):
		self.clientsocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.clientsocket.connect((ip, port))
		self.ip = ip

		self.actionclient = actionlib.SimpleActionClient('move_base', MoveBaseAction)
		self.actionclient.wait_for_server()

	def pose_callback(self, msg):
		pose = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
				msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z,
				msg.pose.orientation.w)

		self.clientsocket.send("('" + self.ip + "'," + str(pose) + "," + feedback + ")")

	def connectionMade(self):
		print "a client connected"
		self.factory.clients.append(self)
		print "clients are ", self.factory.clients

	def connectionLost(self, reason):
		self.factory.clients.remove(self)

	def dataReceived(self, data):
		print "Data received"
		print data

		frame_id, pose = eval(data)

		goal.target_pose.header.frame_id = frame_id
		goal.target_pose.header.stamp = rospy.Time.now()

		goal.target_pose.pose.position.x = pose[0]
		goal.target_pose.pose.position.y = pose[1]
		goal.target_pose.pose.position.z = pose[2]

		goal.target_pose.pose.orientation.x = pose[3]
		goal.target_pose.pose.orientation.y = pose[4]
		goal.target_pose.pose.orientation.z = pose[5]
		goal.target_pose.pose.orientation.w = pose[6]

		self.actionclient.sendGoal(goal)

		client.wait_for_result()

		# feedback = client.get_result()
		feedback = "success"

		self.clientsocket.send("('" + self.ip + "', (None, None, None)," + feedback + ")")



if __name__ == '__main__':
	ip = sys.argv[1]
	port_listen = int(sys.argv[2])
	port_talk = int(sys.argv[3])

	rospy.init_node('turtling', anonymous=True)

	turtling = Turtling(ip, port_talk)

	# need subscriber for pose
	rospy.Subscriber("/amcl_pose", String, lambda x: turtling.pose_callback(x))

	# Start the server/reactor loop
	factory = Factory()
	factory.protocol = turtling
	factory.clients = []
	reactor.listenTCP(port_listen, factory)
	reactor.run()