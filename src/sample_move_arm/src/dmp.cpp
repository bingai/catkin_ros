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

Dmp()
{

}


bool ReceiveMarkerArray(geometry_msgs::PoseArray)
{
	data_assigned  =true;
    for(int i=0; i<ar.poses.size(); i++)
    {
      waypoints.push_back(ar.poses[i]);
    }

  }
  // std::cout<<" I heard "<<ar.poses[0].position.x<<" "<<ar.poses[0].position.y<<" "<<ar.poses[0].position.z<<std::endl;
}

void LoadPosestamped()
{

}

void ComputePhaseFunction()
float interpolated_y(float x1, float x2, float y1, float y2)

