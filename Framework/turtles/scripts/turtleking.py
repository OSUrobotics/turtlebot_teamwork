#!/usr/bin/env/python

import rospy

from move_base_msgs.msg import MoveBase
from std_msgs.msg import Int
from geometry_msgs.msg import Pose

from twisted.internet import reactor
from twisted.internet.protocol import Factory, Protocol

class Turtleking(Protocol):
	def __init__(self, ip, port):
		self.ip = ip

		self.client_number = 0
		self.d = dict()
		self.pending = {}
		self.active = {}
		self.clients = {}

		self.turtles_pub = rospy.Publisher("/turtles", Int)

	def connectionMade(self):
		print "a client connected"
		self.factory.clients.append(self)
		print "clients are ", self.factory.clients
		self.client_number = self.client_number+1

		self.clients[self] = self.client_number

		self.turtles_pub.publish(self.client_number)

	def connectionLost(self, reason):
		self.factory.clients.remove(self)
		self.turtles_pub.publish(-self.clients[self])

	def dataReceived(self, data):
		print "Data received"
		print data
		client_ip, pos, feedback = eval(data)
		print client_ip
		print pos
		print feedback
		key = client_ip

		if key in self.d.keys():
			if feedback == "success":
				for val in self.active.values():
					if val != None:
						done = False
					else:
						done = True

				if done:
					for (key, value) in self.d.items():
						if len(self.pending[key]) > 0:
							value[3].send("('" + self.pending[key][0][1] + "'," + self.pending[key][0][1] + ")")
							self.active[key] = self.pending[key][0]
							self.pending[key] = self.pending[key][1:]
						else:
							self.active[key] = None

			elif feedback == "failure":
				print "oh shit!"

			elif feedback == None:
				self.d[key][0] = pos
				self.publish_pose(self.d[key])

			else:
				print "also, oh shit!"

		else:
			frame_id = "turtlebot_%d" % client_number

			sub = rospy.Subscriber(frame_id + "/nav_goal", MoveBase, lambda x: self.add_goal(x, client_ip))
			pub_pose = rospy.Publisher(frame_id + "/amcl_pose", Pose)
			# pub_feedback = rospy.Publisher()
			connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
			connection.connect((ip, port))

			data = [pos,frame_id,sub,connection, pub_pose]
			self.d[client_ip] = data

	def add_goal(self, msg, client_id):
		frame_id = msg.target_pose.header.frame_id

		pose = (msg.target_pose.pose.position.x,
				msg.target_pose.pose.position.y,
				msg.target_pose.pose.position.z,
				msg.target_pose.pose.orientation.x,
				msg.target_pose.pose.orientation.y,
				msg.target_pose.pose.orientation.z,
				msg.target_pose.pose.orientation.w)

		self.pending[client_id].append((seq, frame_id, pose))
		# double check this!!!!!
		self.pending[client_id].sort(key=lambda x: x[0])

		

	def publish_pose(self, data):
		pose = Pose()

		pose.position.x = data[0][0]
		pose.position.y = data[0][1]
		pose.position.z = data[0][2]
		pose.orientation.x = data[0][3]
		pose.orientation.y = data[0][4]
		pose.orientation.z = data[0][5]
		pose.orientation.w = data[0][6]

		data[4].publish(pose)


if __name__ == '__main__':
	ip = sys.argv[1]
	port = int(sys.argv[2])

	rospy.init_node('turtleking', anonymous=True)

	turtleking = Turtleking(ip, port_talk)

	# Start the server/reactor loop
	factory = Factory()
	factory.protocol = turtleking
	factory.clients = []
	reactor.listenTCP(port_listen, factory)
	reactor.run()