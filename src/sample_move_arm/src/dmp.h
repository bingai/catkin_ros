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
using namespace std;

Class Dmp{
public:
	Dmp();
	void ComputePhaseFunction(geometry_msgs::PoseStamped);
	void set_start_time(geometry_msgs::PoseStamped);

private:
	float tau;
	float alpha;
	float c_phase;
	time start_time;
	time end_time;

}