#!/usr/bin/env/python

from twisted.internet import reactor
from twisted.internet.protocol import Factory, Protocol

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
    # a = data.split(':')
    # print a
    # if len(a) > 1:
    #     command = a[0]
    #     content = a[1]

    #     msg = ""
    #     if command == "iam":
    #         self.name = content
    #         msg = self.name + " has joined"

    #     elif command == "msg":
    #         msg = self.name + ": " + content
    #         print msg

    #     for c in self.factory.clients:
    #         c.message(msg)

    def message(self, message):
        self.transport.write(message + '\n')


def main():
	# Start the server/reactor loop

	factory = Factory()
	factory.protocol = RobotComm
	factory.clients = []
	reactor.listenTCP(11311, factory)
	reactor.run()

if __name__ == '__main__':
	main()