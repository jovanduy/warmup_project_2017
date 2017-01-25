#!/usr/bin/env python

""" This script is our first ROS node. We'll publish some messages """

from geometry_msgs.msg import Point, PointStamped, PointStamped, Pose, Vector3
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA
import rospy

rospy.init_node('test_marker')

r = rospy.Rate(2)
loop_count = 0;

publisher = rospy.Publisher('/my_topic', Marker, queue_size=10)

while not rospy.is_shutdown():
	print "looping", loop_count
	loop_count += 1
	my_header = Header(loop_count, rospy.Time.now(), "base_link")
	my_point = Point(1.0, 2.0, 0.0)
	my_point_stamped = PointStamped(my_header, my_point)
	
	my_pose = Pose(position=my_point)
	my_scale = Vector3(1.0,1.0,1.0)
	my_color = ColorRGBA(0, 1.0, 0, 1.0)

	my_marker = Marker(header=my_header, pose=my_pose, type=2, color=my_color, scale=my_scale)
	print my_marker
	publisher.publish(my_marker)

	r.sleep()


print "Node is finished" 