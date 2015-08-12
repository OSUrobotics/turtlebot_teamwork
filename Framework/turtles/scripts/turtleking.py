#!/usr/bin/env/python

from twisted.internet import reactor
from twisted.internet.protocol import Factory, Protocol

class RobotComm(Protocol):
    def __init__(self):
        client_number = 0
        d = dict()
    def connectionMade(self):
        print "a client connected"
        self.factory.clients.append(self)
        print "clients are ", self.factory.clients
        client_number = client_number+1

    def connectionLost(self, reason):
        self.factory.clients.remove(self)
        client_number = client_number-1


    def dataReceived(self, data):
    	print "Data received"
    	print data
    	ip, pos = eval(data)
        print ip
        print pos
        key = ip
    for i in xrange(100):
        if key in d:
            frame_id = d[key][1]
            data = [pos,frame_id]
            dict[ip] = (data)
        else:
            frame_id = "turtlebot_%d" % client_number
            data = [pos,frame_id]
            dict[ip] = (data)
        
        

    def message(self, message):
        self.transport.write(message + '\n')


def main():
	# Start the server/reactor loop

	factory = Factory()
	factory.protocol = RobotComm
	factory.clients = []
	reactor.listenTCP(8089, factory)
	reactor.run()

if __name__ == '__main__':
	main()