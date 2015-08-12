#!/usr/bin/env python

from geometry_msgs.msg import Pose, Point, Quaternion
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


if __name__ == '__main__':
    print 'Lets make some formations!'
    D_tb = 0.352  # turtlebot diameter (m)
    D_tb *= 1.10  # pad so they don't collide    

    # Make a line
    root_pose = Pose()
    line_form = Lineform(N=4, 
                         D=D_tb,
                         root=root_pose)

    # Drive a line around
    for position, angle in zip([p * 0.50 for p in range(20)], 
                               [a * pi/60 for a in range(20)]):
        quaternion_vector = quaternion_about_axis(angle, z_axis)
        line_form.root.position = Point(position,position,0)
        line_form.root.orientation = Quaternion(*quaternion_vector)
        line_form.update()
        line_form.graph()


    # Make a circle
    root_pose = Pose()
    circle_form = Circleform(N=4, 
                             D=D_tb,
                             root=root_pose)

    # Drive a circle around
    for position, angle in zip([p * 0.50 for p in range(20)], 
                               [a * pi/60 for a in range(20)]):
        quaternion_vector = quaternion_about_axis(angle, z_axis)
        circle_form.root.position = Point(position,position,0)
        circle_form.root.orientation = Quaternion(*quaternion_vector)
        circle_form.update()
        circle_form.graph()

    
