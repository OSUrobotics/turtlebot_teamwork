#!/usr/bin/env python

import socket
import time
import random


class Turtling:
	def __init__(self, ip, port):
		self.clientsocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.clientsocket.connect((ip, port))
		self.ip = "123.123.123.123"
		self.id = random.randint(1, 100000)

	def pose_callback(self, msg):
		pose = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

		self.clientsocket.send(self.ip + str(pose) + str(self.id))





clientsocket.connect(('10.214.152.40', 8089))
# clientsocket.connect(('10.214.152.185', 12397))

clientsocket.send('("132.123.123.123",(1,2,3),533651)')

clientsocket.close()
