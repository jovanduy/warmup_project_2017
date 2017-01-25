#!/usr/bin/env python

""" Teleop control from keyboard to Neato."""

import tty
import select
import sys
import termios
from geometry_msgs.msg import Point, PointStamped, PointStamped, Pose, Vector3, Twist, Transform, PoseWithCovariance, TwistWithCovariance
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Header
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseClient
import rospy

rospy.init_node('teleop_drive')

r = rospy.Rate(2)
loop_count = 0;



my_header = Header(loop_count, rospy.Time.now(), "base_link")
publisher = rospy.Publisher('/my_topic', Transform, queue_size=10)
my_position = Point(0, 0, 0)
my_pose = Pose(position=my_position)


def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

settings = termios.tcgetattr(sys.stdin)
key = None


goal_position = my_position

while key != '\x03':
    key = getKey()
    if key == 'i':
    	goal_position = Point(1, 0, 0)
    	my_goal_pose = Pose(position=goal_position)
    if key == 'k':
    	goal_position = my_position
    	my_goal_pose = my_pose

    
    my_goal = MoveBaseGoal(target_pose = my_goal_pose)
    MoveBaseAction(my_goal)
    


    print key
    publisher.publish()

