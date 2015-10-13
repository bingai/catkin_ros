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
#include <sample_move_arm/dmp.h>
#include <sample_move_arm/PoseStampedArray.h>
using namespace std;

Dmp::Dmp()
{
	// waypoints_ = wp;
}

bool Dmp::get_data_assigned()
{
	return data_assigned_;
}

void Dmp::set_data_assigned(bool v){
	data_assigned_ = v;
}

void Dmp::set_waypoints(std::vector<geometry_msgs::Pose> wp, std::vector<geometry_msgs::PoseStamped> wps){
	waypoints_ = wp;
	waypoints_stamped_ = wps;
	cout<<"Set"<<endl;
}


// void ComputePhaseFunction()
// float interpolated_y(float x1, float x2, float y1, float y2)

