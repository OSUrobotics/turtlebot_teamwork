#!/usr/bin/env python

import socket
import time
import random
import sys
import rospy
import actionlib
from geometry_msgs.msg import Pose
from std_msgs.msg import String

from twisted.internet import reactor
from twisted.internet.protocol import Factory, Protocol


class Turtling(Protocol):
	def __init__(self, ip, port):
		self.clientsocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.clientsocket.connect((ip, port))
		self.ip = ip
		self.id = random.randint(1, 100000)

		self.actionclient = actionlib.SimpleActionClient('move_base', actionlib_tutorials.msg.FibonacciAction)

	def pose_callback(self, msg):
		pose = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

		self.clientsocket.send("(" + self.ip + "," + str(pose) + ")")

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

client = actionlib.SimpleActionClient('fibonacci', actionlib_tutorials.msg.FibonacciAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = actionlib_tutorials.msg.FibonacciGoal(order=20)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()



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