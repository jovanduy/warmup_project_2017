#!/usr/bin/env python

""" Teleop control from keyboard to Neato."""

import tty
import select
import sys
import termios
from geometry_msgs.msg import Vector3, Twist
import rospy

class TeleopNode(object):
    def __init__(self):
        """Initializes a rospy node, as well as a the rate and publisher.""""
        rospy.init_node('teleop_drive')
        self.r = rospy.Rate(2)
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.settings = termios.tcgetattr(sys.stdin)
        self.key = None

    def setKey(self):
        """We were given this function by Paul. It allows us to detect which key is entered."""
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        self.key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
    
    def run(self):
        """When a keystroke is detected, we create and then publish the appropriate twist."""
        while self.key != '\x03':
            self.setKey()
            if self.key == 'u':
                twist = Twist(linear=Vector3(0.5, 0.0, 0.0), angular=Vector3(0.0, 0.0, 1.0))
            elif self.key == 'i':
    	        twist = Twist(linear=Vector3(0.5, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0))
            elif self.key == 'o':
                twist = Twist(linear=Vector3(0.5, 0.0, 0.0), angular=Vector3(0.0, 0.0, -1.0))
            elif self.key == 'j':
                twist = Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 1.0))
            elif self.key == 'k':
                twist = Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0))
            elif self.key == 'l':
                twist = Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, -1.0))
            elif self.key == 'm':
                twist = Twist(linear=Vector3(-0.5, 0.0, 0.0), angular=Vector3(0.0, 0.0, -1.0))
            elif self.key == ',':
                twist = Twist(linear=Vector3(-0.5, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0))
            elif self.key == '.':
                twist = Twist(linear=Vector3(-0.5, 0.0, 0.0), angular=Vector3(0.0, 0.0, 1.0))
            self.publisher.publish(twist)
            self.r.sleep()
            
if __name__ == "__main__":
    teleop_node = TeleopNode()
    teleop_node.run()
