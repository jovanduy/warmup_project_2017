#!/usr/bin/env python

""" Node to make Neato drive in a square. """

from geometry_msgs.msg import Vector3, Twist
import rospy
import math

class SquareNode(object):
    def __init__(self):
        self.r = rospy.Rate(5)
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.time = rospy.Time.now()
        self.states = ['forward', 'turning']
        self.curr_state = self.states[0]
	self.twist = None

    def stop(self):
        self.publisher.publish(Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0)))


        
    def turn_left(self):
        if (rospy.Time.now() - self.time >= rospy.Duration(3)):
            self.time = rospy.Time.now()
            return self.go_forward
        self.twist= Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.5))
        self.publisher.publish(self.twist)
        return self.turn_left


    
    def go_forward(self):
        if (rospy.Time.now() - self.time >= rospy.Duration(2)):
            self.time = rospy.Time.now()
            return self.turn_left
        self.twist = Twist(linear=Vector3(0.5, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0))
        self.publisher.publish(self.twist)
        return self.go_forward


        
    def run(self):
        rospy.on_shutdown(self.stop)
        curr_state = self.go_forward
        while not rospy.is_shutdown():           
            curr_state = curr_state()
            self.r.sleep()
            
if __name__ == '__main__':
    rospy.init_node('square')
    square_node = SquareNode()
    square_node.run()
