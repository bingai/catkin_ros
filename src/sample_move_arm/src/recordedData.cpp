#include <string>
#include <geometry_msgs/Pose.h>
#include <boost/foreach.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <ar_track_alvar/AlvarMarkers.h>

const std::string topic = "ar_pose_marker";
const std::string filepath = "/home/rahul/git/catkin_ws/src/sample_move_arm/bag/test2.bag";

class KinectBag
{
	public:
		std::vector<geometry_msgs::Pose> getWaypoints();
		KinectBag();
		~KinectBag();
		geometry_msgs::Pose getOrigin();
	private:
		rosbag::Bag bag;
};

KinectBag::KinectBag()
{
	bag.open(filepath, rosbag::bagmode::Read);
	return;
}

KinectBag::~KinectBag()
{
	bag.close();
	return;
}

geometry_msgs::Pose KinectBag::getOrigin()
{
	geometry_msgs::Pose origin;
	rosbag::View view(bag, rosbag::TopicQuery(topic));
	BOOST_FOREACH(rosbag::MessageInstance const m, view)
	{
		ar_track_alvar::AlvarMarkers::ConstPtr markerptr = m.instantiate<ar_track_alvar::AlvarMarkers>();
 		if (markerptr->markers.size() != 0)
            {
                return markerptr->markers[0].pose.pose;
            }
	}
	
}

std::vector<geometry_msgs::Pose> KinectBag::getWaypoints()
{
	std::vector<geometry_msgs::Pose> waypoints;

	geometry_msgs::Pose target_pose3;
	target_pose3.orientation.w = 1.0;
	target_pose3.position.x = 1.0;
	target_pose3.position.y = 1.0;
	target_pose3.position.z = 1.0;
	waypoints.push_back(target_pose3);

	rosbag::View view(bag, rosbag::TopicQuery(topic));

	BOOST_FOREACH(rosbag::MessageInstance const m, view)
	{
		ar_track_alvar::AlvarMarkers::ConstPtr markerptr = m.instantiate<ar_track_alvar::AlvarMarkers>();
 		if (markerptr->markers.size() != 0)
            {
                waypoints.push_back(markerptr->markers[0].pose.pose);  
            }
	}
	return waypoints;
}

std::vector<geometry_msgs::Pose> KinectBag::getTransformedWaypoints()
{
	geometry_msgs::Pose origin = getOrigin();
	vector<geometry_msgs::Pose> waypoints = getWaypoints();
	
}