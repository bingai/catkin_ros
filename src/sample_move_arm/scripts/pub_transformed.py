#!/usr/bin/env python
import rosbag
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from sample_move_arm.msg import PoseStampedArray
import geometry_msgs
import tf
import rospy
from bagger import Bagger
import constants
# from std_msgs.msg import String

def pub_transformed():
	pub = rospy.Publisher('pub_transformed', PoseStampedArray, queue_size=10)
	rospy.init_node('pub_transformed', anonymous=True)
	rate = rospy.Rate(1) # 10hz
	b = Bagger(filename=constants.DEMO_FILE)
	posarr = PoseStampedArray()
	posarr.header.frame_id = 'r_wrist_roll_link'
	posarr.header.stamp = rospy.get_rostime()

	posarr.poses = b.getTransformedPosesStamped()
	# print posarr
	while not rospy.is_shutdown():
		pub.publish(posarr)
		rate.sleep()
		# print "publishing transformed_waypoints_posestamped"


if __name__ == '__main__':
	try:
		pub_transformed()
		# b = Bagger()
		# b.getTransformedWaypointsPoses()
	except rospy.ROSInterruptException:
		pass
