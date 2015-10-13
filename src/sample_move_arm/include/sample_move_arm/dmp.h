#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <iostream>
#include <vector>
#include <std_msgs/Time.h>
using namespace std;


class Dmp{
public:
	Dmp();
	// void ReceivePSArray(sample_move_arm::PoseStampedArray);
	
	bool get_data_assigned();
	void set_waypoints(std::vector<geometry_msgs::Pose>, std::vector<geometry_msgs::PoseStamped>);
	void set_start_time(geometry_msgs::PoseStamped);
	void set_data_assigned(bool);
	
	void ComputePhaseFunction(geometry_msgs::PoseStamped);
private:
	float tau_;
	float alpha_;
	float c_phase_;
	std_msgs::Time start_time_;
	std_msgs::Time end_time_;
	vector<geometry_msgs::PoseStamped> waypoints_stamped_;
	vector<geometry_msgs::Pose> waypoints_;
	bool data_assigned_;
};