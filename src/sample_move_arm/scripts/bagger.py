#!/usr/bin/env python
import rosbag
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
import geometry_msgs
import tf
import rospy
import constants

def rosPoseToVec(pose):
	vec = [0]*7
	vec[0] = pose.position.x
	vec[1] = pose.position.y
	vec[2] = pose.position.z
	vec[3] = pose.orientation.x
	vec[4] = pose.orientation.y
	vec[5] = pose.orientation.z
	vec[6] = pose.orientation.w
	return vec

def vecToRosTransform(vec): 
	trans = geometry_msgs.msg.Transform()
	trans.translation.x = vec[0]
	trans.translation.y = vec[1]
	trans.translation.z = vec[2]
	trans.rotation.x = vec[3]
	trans.rotation.y = vec[4]
	trans.rotation.z = vec[5]
	trans.rotation.w = vec[6]
	return trans

class Bagger:
	def __init__(self, filename, topic='ar_pose_marker'):
		self.topic = topic
		self.filename = filename
		self.bag = rosbag.Bag(filename, 'r')
		self.tfl = tf.TransformListener()

	def getOrigin(self):
		p = Pose()
		for topic, msg, t in self.bag.read_messages(topics=[self.topic]):
			if len(msg.markers):
				return msg.markers[0].pose.pose

	#returns markers ar markers
	def getMarkers(self):
		waypoints = []
		count = 0
		obj_once = False

		for topic, msg, t in self.bag.read_messages(topics=[self.topic]):
			if len(msg.markers):
				if msg.markers[0].id == 0:
					waypoints.append(msg.markers[0])
					# print msg.markers[0]
				count+=1
		return waypoints

	#returns markers ar markers for shobhit
	# def getMarkers(self):
	# 	waypoints = []
	# 	count = 0
	# 	obj_once = False

	# 	for topic, msg, t in self.bag.read_messages(topics=[self.topic]):
	# 		# if not obj_once:
			
	# 		# print msg
	# 		# if msg.markers[0].id ==1:
	# 			# obj_once = True
	# 		if len(msg.markers):
	# 			if msg.markers[1].id == 1:
	# 				# print 'in'
	# 				waypoints.append(msg.markers[1])
	# 				# print msg.markers[0]
	# 			count+=1
	# 	# print count
	# 	return waypoints


	def getTransformedPosesStamped(self):
		#sends waypoints with time alogn with them. PoseStamped
		origin_marker = self.getOrigin()
		# print origin_marker
		tformer = tf.TransformerROS(True, rospy.Duration(10.0))

		#Make a transformation from torso to the current wrist pose
		m = geometry_msgs.msg.TransformStamped()
		m.header.frame_id = 'camera_link'
		m.child_frame_id = 'r_wrist_roll_link'
		m.transform = vecToRosTransform(rosPoseToVec(origin_marker))
		tformer.setTransform(m)

		# n = geometry_msgs.msg.TransformStamped()
		# n.header.frame_id = 'camera_link'
		# n.child_frame_id = 'r_wrist_roll_link'
		# n.transform = vecToRosTransform(rosPoseToVec(origin_marker))
		# tformer.setTransform(m)

		recorded_waypoints = self.getMarkers()
		
		waypoints = []
		
		count = 0
		for wp in recorded_waypoints:
			wpstamped = wp.pose
			wpstamped.header.frame_id = wp.header.frame_id
			twp = tformer.transformPose('r_wrist_roll_link', wpstamped)
			# twp = tformer.transformPose('odom_combined', twp)
			twp.header.stamp.secs = wp.header.stamp.secs
			# twp.header.stamp.secs /=30

			twp.header.stamp.nsecs = wp.header.stamp.nsecs
			# print twp.header.stamp
			waypoints.append(twp)
			count+=1
		return waypoints

	#ar_track markers. for plotting
	def getTransformedMarkers(self):
		#for plotting
		origin_marker = self.getOrigin()
		# print origin_marker
		tformer = tf.TransformerROS(True, rospy.Duration(10.0))
		#Make a transformation from torso to the current wrist pose
		m = geometry_msgs.msg.TransformStamped()
		m.header.frame_id = 'camera_link'
		m.child_frame_id = 'r_wrist_roll_link'
		m.transform = vecToRosTransform(rosPoseToVec(origin_marker))
		tformer.setTransform(m)

		recorded_waypoints = self.getMarkers()
		
		waypoints = []
		
		# count = 0
		for wp in recorded_waypoints:
			wpstamped = wp.pose
			wpstamped.header.frame_id = wp.header.frame_id
			twp = tformer.transformPose('r_wrist_roll_link', wpstamped)
			wp.pose = twp
			wp.header.frame_id = wp.pose.header.frame_id
			waypoints.append(wp)
			# count+=1

		return waypoints


	#returns posestamped
	def transformStartGoal(self, state_markers):
		#for plotting
		origin_marker = self.getOrigin()
		# print origin_marker
		tformer = tf.TransformerROS(True, rospy.Duration(10.0))
		#Make a transformation from torso to the current wrist pose
		m = geometry_msgs.msg.TransformStamped()
		m.header.frame_id = 'camera_link'
		m.child_frame_id = 'r_wrist_roll_link'
		m.transform = vecToRosTransform(rosPoseToVec(origin_marker))
		tformer.setTransform(m)		
		
		waypoints = []
		
		# count = 0
		for wp in state_markers:
			wpstamped = wp.pose
			wpstamped.header.frame_id = wp.header.frame_id
			twp = tformer.transformPose('r_wrist_roll_link', wpstamped)
			wp.pose = twp
			wp.header.frame_id = wp.pose.header.frame_id
			waypoints.append(wp.pose.pose)
		return waypoints

	# def getTransformedPoses(self):
	# 	#converts pose stamped to pose
	# 	transformed_waypoints = self.getTransformedPosesStamped()
	# 	waypoints = []
	# 	count =1
	# 	for twp in transformed_waypoints:
	# 		waypoints.append(twp.pose)
	# 	return waypoints

# if __name__ == '__main__':
# 	try:
# 		b = Bagger(constants.DEMO_FILE)
# 		b.getMarkers()

# 	except rospy.ROSInterruptException:
# 		pass


