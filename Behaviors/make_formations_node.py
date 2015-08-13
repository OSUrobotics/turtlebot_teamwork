#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from std_msgs.msg import *
from move_base_msgs.msg import *
from matplotlib import pyplot
from math import sin, cos, pi
from tf.transformations import *
import copy  # :-(

z_axis = (0,0,1)

class Formation(object):  # needs to inherit from "object" class to be a "new-style" class in Python. See StackOverflow for hubbub.
    def __init__(self, N, D, root):
        """ N = number of robots.
            D = diameter of agent for spacing. 
            root = reference point (ROS Point msg). """
        self.N = N
        self.D = D
        self.root = root
        self.spots = []
        for n in range(N):  # initialize agents at "root" location
            self.spots.append(copy.deepcopy(self.root.position))

    def rotate_once(self):
        # Rotate to desired orientation
        orientation_quaternion = tuple(self.root.orientation.__getattribute__(dim) 
                                       for dim in 'xyzw')
        orientation_angle = euler_from_quaternion(orientation_quaternion)[2]
        root_vector = tuple(self.root.position.__getattribute__(dim) 
                            for dim in 'xyz')
        R = rotation_matrix(orientation_angle,
                            z_axis,
                            root_vector)
        for spot in self.spots:
            spot_vector = [spot.__getattribute__(dim) 
                           for dim in 'xyz']
            spot_vector.append(1.)  # add 4th element for matrix multiplication
            spot_vector = numpy.array(spot_vector)
            spot_vector = numpy.expand_dims(spot_vector, 1)
            spot_vector_rotated = R.dot(spot_vector).T[0].tolist()[:3]  # matrix multiplication and formatting
            spot.x = spot_vector_rotated[0]
            spot.y = spot_vector_rotated[1]
        

    def graph(self):
        fig = pyplot.gcf()
        pyplot.ion()
        pyplot.cla()
        for n, spot in enumerate(self.spots):
            if n == 0:
                c = 'r'
            else:
                c = 'k'
            circle = pyplot.Circle((spot.x, spot.y), self.D/2, color=c)
            fig.gca().add_artist(circle)
            
        pyplot.axis('equal')
        pyplot.axis([-10, 10, -10, 10])
        pyplot.grid()
        pyplot.draw()
        #pyplot.show()

class Circleform(Formation):
    def __init__(self, N, D, root):
        super(Circleform, self).__init__(N, D, root)
        self.update()

    def update(self):
        """ For the first time...
        ...or after updating "root"...
        ...or anytime! """
        self.update_circle()
        self.rotate_once()
    
    def update_circle(self):
        tightness = 0.50  # from 0->1, fraction of circle circumference to be occupied by agents...approximately
        circumference = self.N * self.D / tightness
        self.radius = circumference / (2*pi)
        self.center = copy.deepcopy(self.root.position)
        self.center.x -= self.radius  # "root" agent will be at the +x-most point of the circle
        angle_increment = 2*pi/self.N
        self.angles = [0 + i * angle_increment for i in range(self.N)]

        # Place the agents in a circle with default orientation
        for spot, angle in zip(self.spots, self.angles):
            spot.x = self.center.x + self.radius*cos(angle)
            spot.y = self.center.y + self.radius*sin(angle)


class Pyramidform(Formation):
    pass

class Lineform(Formation):
    def __init__(self, N, D, root):
        super(Lineform, self).__init__(N, D, root)
        self.update()

    def update(self):
        """ For the first time...
        ...or after updating "root"...
        ...or anytime! """
        self.update_line()
        self.rotate_once()
        
    def update_line(self):
        tightness = 0.50  # from 0->1, fraction of line length to be occupied by agents
        self.spacing = self.D / tightness
        self.distances = [0 + i * self.spacing for i in range(self.N)]

        # Place the agents in a line with default orientation
        for spot, distance in zip(self.spots, self.distances):
            spot.x = self.root.position.x - distance
            spot.y = self.root.position.y



# GLOBALS! nasty.

D_tb = 0.352  # turtlebot diameter (m)
D_tb *= 1.10  # pad so they don't collide    

# Make a line
line_form = Lineform(N=4, 
                     D=D_tb,
                     root=Pose())

# Make a circle
circle_form = Circleform(N=4, 
                         D=D_tb,
                         root=Pose())

class FormationNode:
    def __init__(self, N=0):
        self.N = N
        self.topics = {}
        for n in range(N):  # initialize agents at "root" location
            self.topics[n+1] = rospy.Publisher("turtlebot_{}/nav_goals".format(n+1), MoveBaseGoal, queue_size=10)
        self.sequence = 0
        
        # Start ROS stuff
        rospy.init_node('make_formations')
        rospy.Subscriber('/tablet_command', Float64MultiArray, self.command_callback)
        #self.command_callback([0,10,10,0,0])  # send fake CIRCLE command
        #self.command_callback([2,0,0,0,1.])  # send fake DIRECTION command


    def send_goals(self, spots):
        header = Header(self.sequence, rospy.Time.now(), '/map')
        self.sequence += 1
        goals = []
        for spot in spots:
            goal = MoveBaseGoal()
            goal.target_pose = PoseStamped(header, Pose(Point(spot.x, spot.y, 0), Quaternion(0, 0, 0, 1)))
            goals.append(goal)
        for t in self.topics:
            if goals:
                self.topics[t].publish(goals.pop(0))

    def change_robot(self, robot_id):
        if robot_id > 0:
            self.topics[robot_id] = rospy.Publisher("turtlebot_{}/nav_goals".format(robot_id), MoveBaseGoal, queue_size=10)
            self.N += 1
        elif robot_id < 0:
            del self.topics[0-robot_id]
            self.N -= 1

    def command_callback(self, cmd):
        #behavior_type, x, y, z, theta = cmd  # parse
        behavior_type, x, y, z, theta = cmd.data  # parse
        if behavior_type == 0:  # "CIRCLE" (for re-positioning the circle)
            print 'circle'
            print cmd.data
            circle_form.root.position.x = x
            circle_form.root.position.y = y
            circle_form.update()
            circle_form.graph()
            self.send_goals(circle_form.spots)
            
        if behavior_type == 3:  # "DIRECTION" (for re-orienting the circle)
            print 'direction'
            print cmd.data
            quaternion_vector = quaternion_about_axis(theta, z_axis)
            circle_form.root.orientation = Quaternion(*quaternion_vector)
            circle_form.update()
            circle_form.graph()
            self.send_goals(circle_form.spots)






if __name__ == '__main__':
    node = FormationNode(N=4)  # for now, initialize 4 robots. Later, can add intelligently via node.change_robot()
    rospy.spin()
