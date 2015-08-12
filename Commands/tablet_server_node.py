#!/usr/bin/env/python

import rospy
from std_msgs.msg import Float64MultiArray
from twisted.internet import reactor
from twisted.internet.protocol import Factory, Protocol

## behavior types circle, line, pyramid, direction (dummies for the others)
## location that is comma separated, x, y, z, theta

class RobotComm(Protocol):
    def __init__(self):
        self.publisher = rospy.Publisher('/tablet_command', String)  # ROS Publisher


    def connectionMade(self):
        print "a client connected"
        self.factory.clients.append(self)
        print "clients are ", self.factory.clients

    def connectionLost(self, reason):
        self.factory.clients.remove(self)

    def dataReceived(self, data):
    	print "Data received"
    	print data
       
        
        msg = data.split(',')

        # get behavior type
    	if msg[0] == "CIRCLE":
    		behavior_type = 0
    	elif msg[0] == "LINE":
    		behavior_type = 1
    	elif msg[0] == "PYRAMID":
    		behavior_type = 2
    	elif msg[0] == "DIRECTION":
    		behavior_type = 3
    	elif msg[0] == "KILL":
    		behavior_type = 4
    	else:
    		behavior_type = -1

    	# parse location	
    	x = float(msg[1])
    	y = float(msg[2])
    	z = float(msg[3])
    	theta = float(msg[4])

    	location = (x, y, z, theta)

    	print "Type: ", behavior_type
    	print "Location: ", location

        # Publish via ROS
        msg = Float64MultiArray()
        msg.data = [behavior_type, x, y, z, theta]
        self.publisher.publish(msg)


    def message(self, message):
        self.transport.write(message + '\n')


def main():
        # Run ROS node
        rospy.init_node('tablet_server')

	# Start the server/reactor loop
	factory = Factory()
	factory.protocol = RobotComm
	factory.clients = []
	reactor.listenTCP(11411, factory)
	reactor.run()

if __name__ == '__main__':
	main()
