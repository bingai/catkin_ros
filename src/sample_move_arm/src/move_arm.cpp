#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <boost/foreach.hpp>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <geometry_msgs/PoseArray.h>

#include <sample_move_arm/PoseStampedArray.h>

// geometry_msgs::PoseArray transformedWaypoints;
std::vector<geometry_msgs::PoseStamped> waypoints;
bool data_assigned = false;

void receiveWaypointPoses(sample_move_arm::PoseStampedArray ar)
{
  if(!data_assigned)
  { 
    data_assigned  =true;
    for(int i=0; i<ar.poses.size(); i++)
    {
      waypoints.push_back(ar.poses[i]);
    }
  }
  // std::cout<<" I heard "<<ar.poses[0].position.x<<" "<<ar.poses[0].position.y<<" "<<ar.poses[0].position.z<<std::endl;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_arm");
  ros::NodeHandle node_handle;  
  ros::AsyncSpinner spinner(1);
  spinner.start();
  /* This sleep is ONLY to allow Rviz to come up */
  //sleep(20.0);

  // The :move_group_interface:`MoveGroup` class can be easily 
  // setup using just the name
  // of the group you would like to control and plan for.
  moveit::planning_interface::MoveGroup group("right_arm");

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to deal directly with the world.
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  

  // (Optional) Create a publisher for visualizing plans in Rviz.
  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
  
  // We can also print the name of the end-effector link for this group.
  ROS_INFO("End Effector link: %s", group.getEndEffectorLink().c_str());

  group.setPoseReferenceFrame("r_wrist_roll_link");

  std::cout<<"Should this be planning frame or pose reference frame"<<std::endl;
  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());


  ros::Subscriber sub = node_handle.subscribe("pub_transformed", 1000, receiveWaypointPoses);

  while(!data_assigned)
  {
    sleep(1);
  }

  // Dmp dm_obj = new Dmp();


  // Cartesian Paths
  // ^^^^^^^^^^^^^^^
  // You can plan a cartesian path directly by specifying a list of waypoints 
  // for the end-effector to go through. Note that we are starting 
  // from the new start state above.  The initial pose (start state) does not
  // need to be added to the waypoint list.

// robot_state::RobotState start_state(*group.getCurrentState());
// // // start_state.printStateInfo(std::cout);
// // geometry_msgs::Pose start_pose2;
// // start_pose2.orientation.w = 1.0;
// // start_pose2.position.x = 0;
// // start_pose2.position.y = 0;
// // start_pose2.position.z = 0.55;
// const robot_state::JointModelGroup *joint_model_group =
//                 start_state.getJointModelGroup(group.getName());
// start_state.setFromIK(joint_model_group, start_pose2);
// group.setStartState(start_state);

//   geometry_msgs::Pose target_pose3 = start_pose2;
//   target_pose3.position.x += 0.2;
//   // target_pose3.position.z -= 0.2;
//   waypoints.push_back(target_pose3);  // up and out

//   target_pose3.position.x += 0.1;
//   waypoints.push_back(target_pose3);  // left

//   // target_pose3.position.x += 0.1;
//   // waypoints.push_back(target_pose3);  // left

//     target_pose3.position.y += 0.1;
//   waypoints.push_back(target_pose3);  // left

  //   target_pose3.position.y += 0.1;
  //   waypoints.push_back(target_pose3);  // left

  // target_pose3.position.z -= 0.2;
  // target_pose3.position.y += 0.2;
  // target_pose3.position.x -= 0.2;
  // waypoints.push_back(target_pose3);  // down and right (back to start)

    
  // We want the cartesian path to be interpolated at a resolution of 1 cm
  // which is why we will specify 0.01 as the max step in cartesian
  // translation.  We will specify the jump threshold as 0.0, effectively
  // disabling it.
  moveit_msgs::RobotTrajectory trajectory;
  // double fraction = group.computeCartesianPath(waypoints,
  //                                              0.01,  // eef_step
  //                                              0.0,   // jump_threshold
  //                                              trajectory);

  // ROS_INFO("Executing trajectory (cartesian path)");

  // moveit::planning_interface::MoveGroup::Plan plan;
  // plan.trajectory_ = trajectory;
  // group.execute(plan);
  


  sleep(15.0);

  

  ros::shutdown();  
  return 0;
}