#!/usr/bin/env python

import rospy
from numpy import array
from std_msgs.msg import *
from geometry_msgs.msg import Point, Point32, PoseStamped, Quaternion
from dlp.msg import DefendersState, EnemyState

def main():
	rospy.init_node('defender_node', anonymous=True)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
