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
        self.twist = None

    def stop(self):
        """ This function is called on shutdown and publishes
        a twist with linear and angular velocities of 0 to stop the Neato."""
        self.publisher.publish(Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0)))
        
    def turn_left(self):
        """ This function publishes a twist to make the Neato turn left. """
        if (rospy.Time.now() - self.time >= rospy.Duration(3)):
            # Checks whether enough time has passed for a 90 degree turn to be completed.
            self.time = rospy.Time.now() 
            # Resets time and changes state to new action.
            return self.go_forward
        # Twist with positive angular velocity to turn left.
        self.twist= Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.5))
        self.publisher.publish(self.twist)
        # Current state remains the same.
        return self.turn_left


    
    def go_forward(self):
        """ This function publishes a twist to make the Neato move forward."""
        if (rospy.Time.now() - self.time >= rospy.Duration(2)):
            # Checks whether enough time has passed for the Neato to drive forward 1 meter.
            self.time = rospy.Time.now()
            # Resets time and changes state to new action.
            return self.turn_left
        self.twist = Twist(linear=Vector3(0.5, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0))
        self.publisher.publish(self.twist)
       # Current state remains the same.
        return self.go_forward


        
    def run(self):
        """This function is the main run loop"""
        rospy.on_shutdown(self.stop)
        curr_state = self.go_forward
        while not rospy.is_shutdown():           
            curr_state = curr_state()
            self.r.sleep()
            
if __name__ == '__main__':
    rospy.init_node('square')
    square_node = SquareNode()
    square_node.run()
