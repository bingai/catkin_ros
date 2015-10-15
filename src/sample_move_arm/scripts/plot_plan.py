#!/usr/bin/env python
from __future__ import division
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Pose
import rospy
import math
from bagger import Bagger
import constants
#Intended to be run separately. Looks like needs kinect connected as the frame is otherwise not available on rviz
#can be checked using rostopic echo /visualization_marker_array

def plot_plan():
	topic = 'visualization_plan_marker_array'
	publisher = rospy.Publisher(topic, MarkerArray, queue_size=1000)

	rospy.init_node('plot_plan',anonymous=True)
	r = rospy.Rate(1)

	poses = []
	f = open(constants.PLAN_FILE)
	for line in f:
		tokens = line.split()
		p = Pose()
		p.position.x = float(tokens[0])
		p.position.y = float(tokens[1])
		p.position.z = float(tokens[2])
		p.orientation.x = float(tokens[3])
		p.orientation.y = float(tokens[4])
		p.orientation.z = float(tokens[5])
		p.orientation.w = float(tokens[6])
		poses.append(p)

	# alvar_markers = b.getTransformedMarkers()

	markerArray = MarkerArray()

	i = 0
	l = len(poses)
	for p in poses:
		# if not i:
		# 	print alvar_marker
		marker = Marker()
		marker.header.frame_id = 'r_wrist_roll_link'
		marker.header.stamp = rospy.get_rostime()
		marker.ns = "visualization_markers"
		marker.id = i
		marker.type = marker.SPHERE
		marker.action = marker.ADD
		marker.scale.x = 0.03
		marker.scale.y = 0.03
		marker.scale.z = 0.03
		marker.color.a = 1.0
		# marker.color.r = 1.0
		# marker.color.g = 1.0
		marker.color.r = i / l
		marker.color.g = (l-i)/l
		marker.color.b = 0.0
		
		marker.pose = p

		# We add the new marker to the MarkerArray, removing the oldest
	   # marker from it when necessary
		# if(count > MARKERS_MAX):
		# 	markerArray.markers.pop(0)
		marker.lifetime = rospy.Duration();
		markerArray.markers.append(marker)
		i+=1
	# print markerArray
	while not rospy.is_shutdown():
		publisher.publish(markerArray)
		# print "publishing transf marker arry"
		r.sleep()

if __name__ == '__main__':
	try:
		plot_plan()

	except rospy.ROSInterruptException:
		pass


