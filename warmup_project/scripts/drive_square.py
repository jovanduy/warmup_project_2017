#!/usr/bin/env python

""" Teleop control from keyboard to Neato."""

from geometry_msgs.msg import Vector3, Twist
import rospy
import math

class SquareNode(object):
    def __init__(self):
        rospy.init_node('square')
        self.r = rospy.Rate(2)
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.time = rospy.Time.now()
        self.states = ['forward', 'turning']
        self.curr_state = self.states[0]

    def stop(self):
        self.publisher.publish(Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0)))
        
    def run(self):
        rospy.on_shutdown(self.stop)
        while not rospy.is_shutdown():
            if self.curr_state == self.states[0]:
                twist = Twist(linear=Vector3(0.5, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0))
                if (rospy.Time.now() - self.time >= rospy.Duration(2)):
                    self.time = rospy.Time.now()
                    self.curr_state = self.states[1]
            elif self.curr_state == self.states[1]:
                twist = Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 1.0))
                if (rospy.Time.now() - self.time >= rospy.Duration(1.34)):
                    self.time = rospy.Time.now()
                    self.curr_state = self.states[0]
            self.publisher.publish(twist)
            self.r.sleep()
        
            
square_node = SquareNode()
square_node.run()
