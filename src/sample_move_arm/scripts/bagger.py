#!/usr/bin/env python
import rosbag
import geometry_msgs.msg
import tf
import rospy
# from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('bagger', tuple, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

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
	def __init__(self, topic='ar_pose_marker', filename='/home/rahul/git/ros_pr2/src/move_pr2withkinect/bag/square2.bag'):
		self.topic = topic
		self.filename = filename
		self.bag = rosbag.Bag(filename, 'r')

	def getOrigin(self):
		p = geometry_msgs.msg.Pose()comm
		for topic, msg, t in self.bag.read_messages(topics=[self.topic]):
			if len(msg.markers):
				return msg.markers[0].pose.pose

	#actually returns markers
	def getWaypoints(self):
		waypoints = []
		count = 0
		for topic, msg, t in self.bag.read_messages(topics=[self.topic]):
			if len(msg.markers):
				waypoints.append(msg.markers[0])
				count+=1

		return waypoints

	
	def getTransformedWaypointsPoses(self):
		origin_marker = self.getOrigin()
		# print origin_marker
		tformer = tf.TransformerROS(True, rospy.Duration(10.0))
		#Make a transformation from torso to the current wrist pose
		m = geometry_msgs.msg.TransformStamped()
		m.header.frame_id = 'camera_depth_optical_frame'
		m.child_frame_id = 'end_effector'
		m.transform = vecToRosTransform(rosPoseToVec(origin_marker))
		tformer.setTransform(m)

		recorded_waypoints = self.getWaypoints()
		
		waypoints = []
		
		count = 0
		for wp in recorded_waypoints:
			wpstamped = wp.pose
			wpstamped.header.frame_id = wp.header.frame_id
			twp = tformer.transformPose('end_effector', wpstamped)
			if not count%50:
				waypoints.append(twp.pose)
			count+=1	
		return waypoints

	def getTransformedWaypoints(self):
		origin_marker = self.getOrigin()
		# print origin_marker
		tformer = tf.TransformerROS(True, rospy.Duration(10.0))
		#Make a transformation from torso to the current wrist pose
		m = geometry_msgs.msg.TransformStamped()
		m.header.frame_id = 'camera_depth_optical_frame'
		m.child_frame_id = 'end_effector'
		m.transform = vecToRosTransform(rosPoseToVec(origin_marker))
		tformer.setTransform(m)

		recorded_waypoints = self.getWaypoints()
		
		waypoints = []
		
		count = 0
		for wp in recorded_waypoints:
			wpstamped = wp.pose
			wpstamped.header.frame_id = wp.header.frame_id
			twp = tformer.transformPose('end_effector', wpstamped)
			if not count%50:
				waypoints.append(twp)
			count+=1	
		return waypoints

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
