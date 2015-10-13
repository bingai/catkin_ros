#!/usr/bin/env python
import rosbag
from geometry_msgs.msg import Pose
import geometry_msgs
import tf
import rospy
from std_msgs import msg

def pub_times():
	pub = rospy.Publisher('pub_times', tuple(msg), queue_size=10)
	rospy.init_node('pub_times', anonymous=True)
	rate = rospy.Rate(1) # 10hz
	b = Bagger()
	m = msg()
	posarr.header.frame_id = 'r_wrist_roll_link'
	posarr.header.stamp = rospy.get_rostime()
	posarr.poses = b.getTransformedPoses()
	print posarr
	while not rospy.is_shutdown():
		pub.publish(posarr)
		rate.sleep()
		print "publishing transformed_waypoints"


if __name__ == '__main__':
	try:
		pub_times()
		# b = Bagger()
		# b.getTransformedWaypointsPoses()
	except rospy.ROSInterruptException:
		pass
