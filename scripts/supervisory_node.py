#!/usr/bin/env python

import rospy
from numpy import array
from std_msgs.msg import *
from geometry_msgs.msg import Point, Point32, PoseStamped, Quaternion
from dlp.msg import DefendersState, EnemyState

def main():

	rospy.init_node('supervisory_node', anonymous=True)

	# Publishers
	d_pub = rospy.Publisher('defenders_locations', DefendersState, queue_size=10)
	e_pub = rospy.Publisher('enemy_locations', EnemyState, queue_size=10)
	local_pub = rospy.Publisher('mavros/local_position/pose', PoseStamped, queue_size=10)

	# TODO: Subscribers: to optitrack topics

	# msgs
	d_msg = DefendersState();
	e_msg = EnemyState()
	local_msg = PoseStamped()

	# msg initialization
	d_msg.header.stamp = rospy.Time.now()
	d_msg.defenders_count = 3
	points=[]
	p=Point32(6.5,5.5,1.0)
	points.append(p)
	p=Point32(2.5,4.5,1.0)
	points.append(p)
	p=Point32(4.5,4.5,1.0)
	points.append(p)
	d_msg.defenders_sectors = [14,17,19]
	d_msg.defenders_position = points

	'''
	d_msg.defenders_position[0].x=0.5
	d_msg.defenders_position[0].y=4.5
	d_msg.defenders_position[1].x=2.5
	d_msg.defenders_position[1].y=4.5
	d_msg.defenders_position[2].x=4.5
	d_msg.defenders_position[2].y=4.5
	'''

	e_msg.header.stamp = rospy.Time.now()
	e_msg.enemy_count = 2
	e_msg.enemy_sectors = [28,26]
	points=[]
	p=Point32(6.5,3.5,1.0)
	points.append(p)
	p=Point32(4.5,3.5,1.0)
	points.append(p)
	e_msg.enemy_position = points
	e_msg.is_captured = [False, False]

	local_msg.header.stamp = rospy.Time.now()
	point = Point(6.5,5.5,1.0)
	orientation = Quaternion(0.0,0.0,0.0,1.0)
	local_msg.pose.position = point
	local_msg.pose.orientation = orientation
	#local_msg = point

	# loop rate
	rate = rospy.Rate(50)

	# Main loop
	while not rospy.is_shutdown():

		d_msg.header.stamp = rospy.Time.now()
		e_msg.header.stamp = rospy.Time.now()
		local_msg.header.stamp = rospy.Time.now()

		d_pub.publish(d_msg)
		e_pub.publish(e_msg)
		local_pub.publish(local_msg)

		rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
