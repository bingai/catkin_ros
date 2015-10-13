#!/usr/bin/env python
from __future__ import division
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import math
from transformed_waypoints import Bagger

#Intended to be run separately. Looks like needs kinect connected as the frame is otherwise not available on rviz
#can be checked using rostopic echo /visualization_marker_array

def plot_transformed():
	topic = 'visualization_transformed_marker_array'
	publisher = rospy.Publisher(topic, MarkerArray, queue_size=1000)

	rospy.init_node('plot_transformed',anonymous=True)
	r = rospy.Rate(1)

	b = Bagger()
	alvar_markers = b.getTransformedWaypointsMarkers()

	markerArray = MarkerArray()

	i = 0
	l = len(alvar_markers)
	for alvar_marker in alvar_markers:
		# if not i:
		# 	print alvar_marker
		marker = Marker()
		marker.header.frame_id = alvar_marker.header.frame_id
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
		
		marker.pose = alvar_marker.pose.pose

		# We add the new marker to the MarkerArray, removing the oldest
	   # marker from it when necessary
		# if(count > MARKERS_MAX):
		# 	markerArray.markers.pop(0)
		marker.lifetime = rospy.Duration();
		markerArray.markers.append(marker)
		i+=1

	while not rospy.is_shutdown():
		publisher.publish(markerArray)
		print "publishing transf marker arry"
		r.sleep()
if __name__ == '__main__':
	try:
		plot_transformed()

	except rospy.ROSInterruptException:
		pass


