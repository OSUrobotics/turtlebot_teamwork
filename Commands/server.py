#!/usr/bin/env/python

from twisted.internet import reactor
from twisted.internet.protocol import Factory, Protocol

## behavior types circle, line, pyramid, direction (dummies for the others)
## location that is comma separated, x, y, z, theta

class RobotComm(Protocol):
    def connectionMade(self):
        print "a client connected"
        self.factory.clients.append(self)
        print "clients are ", self.factory.clients

    def connectionLost(self, reason):
        self.factory.clients.remove(self)

    def dataReceived(self, data):
    	print "Data received"
    	print data
       
        # get behavior type
        msg = data.split(',')
    	if msg[0] == "CIRCLE":
    		behavior_type = 0
    	elif msg[0] == "LINE":
    		behavior_type = 1
    	elif msg[0] == "PYRAMID":
    		behavior_type = 2

    	# parse location	
    	x = float(msg[1])
    	y = float(msg[2])
    	z = float(msg[3])
    	theta = float(msg[4])

    	location = (x, y, z, theta)

    	print "Type: ", behavior_type
    	print "Location: ", location

    def message(self, message):
        self.transport.write(message + '\n')


def main():
	# Start the server/reactor loop

	factory = Factory()
	factory.protocol = RobotComm
	factory.clients = []
	reactor.listenTCP(11411, factory)
	reactor.run()

if __name__ == '__main__':
	main()