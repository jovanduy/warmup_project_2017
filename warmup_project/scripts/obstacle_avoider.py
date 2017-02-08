#!/usr/bin/env python

"""  Neato avoids obstacles. """

from geometry_msgs.msg import Vector3, Twist, Point, Pose
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan, PointCloud
from std_msgs.msg import Header, ColorRGBA
import rospy
import mathimport sys

class ObstacleNode(object):
    def __init__(self):
        rospy.init_node('obstacle_avoider')
        self.r = rospy.Rate(5)
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.marker_publisher = rospy.Publisher('/obstacle_topic', Marker, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.process_scan)
        self.twist = 0
        self.my_marker = 0        self.coords = []
        self.time = rospy.Time.now()
        self.obstacle_front = False
        self.obstacle_side = False
        
        
    def stop(self):
        self.publisher.publish(Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0)))
        
    
    def process_scan(self, m):
        self.ranges = m.ranges
        if (self.ranges[0] < 1 and self.ranges[0] != 0):            self.obstacle_front = True
        else:
            self.obstacle_front = False

        print self.ranges[270]
        if (self.ranges[270] < 1 and self.ranges[270] != 0):
            self.obstacle_side = True
        elif self.ranges[270] == 0:
            self.obstacle_side = False
        print self.obstacle_side
                def turn_left(self):
        if (rospy.Time.now() - self.time >= rospy.Duration(3)):
            self.time = rospy.Time.now()
            return self.go_along
        self.twist= Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.5))
        self.publisher.publish(self.twist)
        return self.turn_left        
    def turn_right(self):
        if (rospy.Time.now() - self.time >= rospy.Duration(3)):
            self.time = rospy.Time.now()            return self.move_towards_goal
        self.twist= Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, -0.5))
        self.publisher.publish(self.twist)
        return self.turn_right
        
    def go_forward(self):
        self.twist = Twist(linear=Vector3(1.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0))
        self.publisher.publish(self.twist)
    def go_along(self):
        if (rospy.Time.now() - self.time >= rospy.Duration(1)):
            if not self.obstacle_side:
                self.time = rospy.Time.now()
                return self.turn_right
            else:
                self.go_forward()
                return self.go_along
        else:
            self.publisher.publish(Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0)))
            return self.go_along

    def move_towards_goal(self):
        if self.obstacle_front:
            self.time = rospy.Time.now()
            return self.turn_left
        else:
            self.go_forward()
            return self.move_towards_goal
        
        
    def run(self):
        rospy.on_shutdown(self.stop)
        curr_state = self.move_towards_goal
        while not rospy.is_shutdown():           
            
            curr_state = curr_state()
            print curr_state
              
            self.r.sleep()
        
            
obstacle_node = ObstacleNode()
obstacle_node.run()
