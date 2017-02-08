#!/usr/bin/env python

"""  Neato follows person around. """

from geometry_msgs.msg import Vector3, Twist, Point, Pose
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan, PointCloud
from std_msgs.msg import Header, ColorRGBA
import rospy
import math
import sys

class PersonNode(object):
    def __init__(self):
        
        self.r = rospy.Rate(5)
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.marker_publisher = rospy.Publisher('/marker_topic', Marker, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.process_scan)
        self.twist = 0
        self.my_marker = 0
        self.coords = []
        self.person_point = 0
        self.person_polar = (0, 0)


    def stop(self):
        self.publisher.publish(Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0)))
        

    def visualize(self):
        my_scale = Vector3(0.2,0.2,0.2)
        my_color = ColorRGBA(0, 0.0, 1.0, 1.0)
        my_header = Header(stamp=rospy.Time.now(), frame_id="base_link")
        person_pose = Pose(position=self.person_point)
        if self.person_point != 0:
            self.my_marker = Marker(header=my_header, type=2, pose=person_pose, scale=my_scale, color=my_color)
      
    
    
    def find_person(self):
        # coords is an array of tuples (angle. r)
        sum_angles = 0
        sum_ranges = 0
        point_count = 0
        avg_angle = 0
        avg_range = 0

        for point in self.coords:
            if point[1] != 0 and point[1] < 1.5:
                sum_angles += point[0]
                sum_ranges += point[1]
                point_count += 1

        if point_count != 0:
            avg_angle = sum_angles/point_count
            avg_range = sum_ranges/point_count
        
        self.person_polar = (avg_angle, avg_range)
        
        if avg_angle < 0:
            avg_angle = 360 + avg_angle
        self.person_point = Point(avg_range*math.cos(math.radians(avg_angle)), avg_range*math.sin(math.radians(avg_angle)), 0)
        
    def calculate_velocity(self):

        # robot position 0, 0, 0

        # human position self.person_point
        p1 = 0.09
        p2 = 0.04
        forward_speed = p1*self.person_point.x
        turn_speed = p2* self.person_polar[0]

        if self.person_polar[1] < .75:
        
            forward_speed = 0
        # robot speed?
        self.twist= Twist(linear=Vector3(forward_speed, 0.0, 0.0), angular=Vector3(0.0, 0.0, turn_speed))



    def process_scan(self, m):
        self.ranges = m.ranges
        self.coords = []
        for angle in range(0,30):
            self.coords.append((angle, m.ranges[angle]))
        for angle in range(359,329, -1):
            angle_new = -1*(360-angle)
            self.coords.append((angle_new, m.ranges[angle]))



    def run(self):
        rospy.on_shutdown(self.stop)

        while not rospy.is_shutdown():
            #self.publisher.publish(self.twist)
            self.find_person()
            self.visualize()
            self.calculate_velocity()
            if self.my_marker != 0:
                self.marker_publisher.publish(self.my_marker)
            if self.twist != 0:
                self.publisher.publish(self.twist)
              
            self.r.sleep()
        
if __name__ == '__main__':            
    rospy.init_node('person_follow')
    person_node = PersonNode()
    person_node.run()
