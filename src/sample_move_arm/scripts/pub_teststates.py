#!/usr/bin/env python
import rosbag
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
import geometry_msgs
import tf
import rospy
from bagger import Bagger
import constants

def pub_teststates():
	pub = rospy.Publisher('pub_teststates', PoseArray, queue_size=10)
	rospy.init_node('pub_teststates', anonymous=True)
	rate = rospy.Rate(1) # 10hz

	a = Bagger(filename=constants.TEST_FILE)
	markers = a.getMarkers()
	start_m = markers[0]
	goal_m = markers[-1]
	sg_markers = [start_m, goal_m]

	b = Bagger(filename=constants.DATA_FILE)
	# tranformed_sg_markers = b.transformStartGoal(sg_markers)
	
	posarr = PoseArray()
	posarr.header.frame_id = 'r_wrist_roll_link'
	posarr.header.stamp = rospy.get_rostime()
	posarr.poses = b.transformStartGoal(sg_markers)

	while not rospy.is_shutdown():
		pub.publish(posarr)
		rate.sleep()
		print "publishing transformed_start_goal"


if __name__ == '__main__':
	try:
		pub_teststates()
		# b = Bagger()
		# b.getTransformedWaypointsPoses()
	except rospy.ROSInterruptException:
		pass
