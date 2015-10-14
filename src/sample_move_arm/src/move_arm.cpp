#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <geometry_msgs/PoseArray.h>
#include <sample_move_arm/PoseStampedArray.h>
#include <sample_move_arm/dmp.h>
#include <cmath>

const double tau = 5.0;

using namespace std;

vector<geometry_msgs::PoseStamped> demoPoseStamped;
vector<geometry_msgs::Pose> testPoses;

bool data_assigned = false;
bool test_assigned = false;

void receivePSArray(sample_move_arm::PoseStampedArray ar)
{
  if(!data_assigned)
  { 
    for(int i=0; i<ar.poses.size(); i++)
      demoPoseStamped.push_back(ar.poses[i]);

    data_assigned = true;
  }
  // std::cout<<" I heard "<<ar.poses[0].position.x<<" "<<ar.poses[0].position.y<<" "<<ar.poses[0].position.z<<std::endl;
}

void receiveTestArray(geometry_msgs::PoseArray ar)
{
  if(!test_assigned)
  { 
    for(int i=0; i<ar.poses.size(); i++)
      testPoses.push_back(ar.poses[i]);

    test_assigned = true;
  }
  // std::cout<<" I heard "<<ar.poses[0].position.x<<" "<<ar.poses[0].position.y<<" "<<ar.poses[0].position.z<<std::endl;
}

void ConvertDataToTrajectory(Trajectory &demo)
{
    double start_time = demoPoseStamped.front().header.stamp.toSec();
    for(std::vector<geometry_msgs::PoseStamped>::iterator it = demoPoseStamped.begin(); it!=demoPoseStamped.end(); it++)
    {
      Point pt;
      geometry_msgs::Pose p = (*it).pose;
      pt.coordinates.push_back(p.position.x);
      pt.coordinates.push_back(p.position.y);
      pt.coordinates.push_back(p.position.z);
      pt.coordinates.push_back(p.orientation.x);
      pt.coordinates.push_back(p.orientation.y);
      pt.coordinates.push_back(p.orientation.z);
      pt.coordinates.push_back(p.orientation.w);
      demo.points.push_back(pt);
      demo.times.push_back(((*it).header.stamp.toSec() - start_time));
    }

}
void ConvertTrajectoryToVecPoses(Trajectory &traj, vector<geometry_msgs::Pose> &poses)
{
  for(int i=0; i<traj.points.size();i++)
  {
    geometry_msgs::Pose p;
    p.position.x = traj.points[i].coordinates[0];
    p.position.y = traj.points[i].coordinates[1];
    p.position.z = traj.points[i].coordinates[2];
    p.orientation.x = traj.points[i].coordinates[3];
    p.orientation.y = traj.points[i].coordinates[4];
    p.orientation.z = traj.points[i].coordinates[5];
    p.orientation.w = traj.points[i].coordinates[6];
    poses.push_back(p);
  }
} 

void ConvertPosesToPoints(Point &start, Point &goal)
{
    start.coordinates.push_back(testPoses[0].position.y);
    start.coordinates.push_back(testPoses[0].position.x);
    start.coordinates.push_back(testPoses[0].position.z);
    start.coordinates.push_back(testPoses[0].orientation.x);
    start.coordinates.push_back(testPoses[0].orientation.y);
    start.coordinates.push_back(testPoses[0].orientation.z);
    start.coordinates.push_back(testPoses[0].orientation.w);
    goal.coordinates.push_back(testPoses[1].position.y);
    goal.coordinates.push_back(testPoses[1].position.x);
    goal.coordinates.push_back(testPoses[1].position.z);
    goal.coordinates.push_back(testPoses[1].orientation.x);
    goal.coordinates.push_back(testPoses[1].orientation.y);
    goal.coordinates.push_back(testPoses[1].orientation.z);
    goal.coordinates.push_back(testPoses[1].orientation.w);
  
} 

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_arm");
  ros::NodeHandle node_handle;  
  ros::AsyncSpinner spinner(1);
  spinner.start();
  moveit::planning_interface::MoveGroup group("right_arm");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  
  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;
  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
  ROS_INFO("End Effector link: %s", group.getEndEffectorLink().c_str());
  group.setPoseReferenceFrame("r_wrist_roll_link");
  std::cout<<"Should this be planning frame or pose reference frame"<<std::endl;
  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());

  ros::Subscriber sub = node_handle.subscribe("pub_transformed", 1000, receivePSArray);
  while(!data_assigned)
  {
    sleep(1);
  }

  ros::Subscriber sub2 = node_handle.subscribe("pub_teststates", 1000, receiveTestArray);
  while(!test_assigned)
  {
    sleep(1);
  }


  Trajectory demonstration, plan;
  ConvertDataToTrajectory(demonstration);
  Point start, goal;
  ConvertPosesToPoints(start, goal);

  DmpGroup d;
  

  double K=0.5;
  double D = sqrt(K)*2;
  d.Learning(demonstration, K, D);
  d.Planning(start, goal, tau, 0.5, plan);

  vector<geometry_msgs::Pose> poses;
  ConvertTrajectoryToVecPoses(plan, poses);

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


  // moveit_msgs::RobotTrajectory trajectory;
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

  // vector<double> K, D;
  // for(int i =0; i<demonstration.points.front().coordinates.size(); i++)
  //   {
  //     K.push_back(0.5);
  //     D.push_back(sqrt(K[i])*2);
  //   }



//   ros::init(argc, argv, "move_arm");
//   ros::NodeHandle node_handle;  
//   ros::AsyncSpinner spinner(1);
//   spinner.start();
//   // The :move_group_interface:`MoveGroup` class can be easily 
//   // setup using just the name
//   // of the group you would like to control and plan for.
//   moveit::planning_interface::MoveGroup group("right_arm");

//   // We will use the :planning_scene_interface:`PlanningSceneInterface`
//   // class to deal directly with the world.
//   moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  
//   // (Optional) Create a publisher for visualizing plans in Rviz.

//   ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
//   moveit_msgs::DisplayTrajectory display_trajectory;

//   // Getting Basic Information
//   // ^^^^^^^^^^^^^^^^^^^^^^^^^
//   //
//   // We can print the name of the reference frame for this robot.
//   ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
  
//   // We can also print the name of the end-effector link for this group.
//   ROS_INFO("End Effector link: %s", group.getEndEffectorLink().c_str());

//   group.setPoseReferenceFrame("r_wrist_roll_link");

//   std::cout<<"Should this be planning frame or pose reference frame"<<std::endl;
//   ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());

//   ros::Subscriber sub = node_handle.subscribe("pub_transformed", 1000, receivePSArray);

//   while(!dmp.get_data_assigned())
//   {
//     sleep(1);
//   }



//   // Cartesian Paths
//   // ^^^^^^^^^^^^^^^
//   // You can plan a cartesian path directly by specifying a list of waypoints 
//   // for the end-effector to go through. Note that we are starting 
//   // from the new start state above.  The initial pose (start state) does not
//   // need to be added to the waypoint list.

// // robot_state::RobotState start_state(*group.getCurrentState());
// // // // start_state.printStateInfo(std::cout);
// // // geometry_msgs::Pose start_pose2;
// // // start_pose2.orientation.w = 1.0;
// // // start_pose2.position.x = 0;
// // // start_pose2.position.y = 0;
// // // start_pose2.position.z = 0.55;
// // const robot_state::JointModelGroup *joint_model_group =
// //                 start_state.getJointModelGroup(group.getName());
// // start_state.setFromIK(joint_model_group, start_pose2);
// // group.setStartState(start_state);

// //   geometry_msgs::Pose target_pose3 = start_pose2;
// //   target_pose3.position.x += 0.2;
// //   // target_pose3.position.z -= 0.2;
// //   waypoints.push_back(target_pose3);  // up and out

// //   target_pose3.position.x += 0.1;
// //   waypoints.push_back(target_pose3);  // left

// //   // target_pose3.position.x += 0.1;
// //   // waypoints.push_back(target_pose3);  // left

// //     target_pose3.position.y += 0.1;
// //   waypoints.push_back(target_pose3);  // left

//   //   target_pose3.position.y += 0.1;
//   //   waypoints.push_back(target_pose3);  // left

//   // target_pose3.position.z -= 0.2;
//   // target_pose3.position.y += 0.2;
//   // target_pose3.position.x -= 0.2;
//   // waypoints.push_back(target_pose3);  // down and right (back to start)

    
//   // We want the cartesian path to be interpolated at a resolution of 1 cm
//   // which is why we will specify 0.01 as the max step in cartesian
//   // translation.  We will specify the jump threshold as 0.0, effectively
//   // disabling it.
//   moveit_msgs::RobotTrajectory trajectory;
//   // double fraction = group.computeCartesianPath(waypoints,
//   //                                              0.01,  // eef_step
//   //                                              0.0,   // jump_threshold
//   //                                              trajectory);

//   // ROS_INFO("Executing trajectory (cartesian path)");

//   // moveit::planning_interface::MoveGroup::Plan plan;
//   // plan.trajectory_ = trajectory;
//   // group.execute(plan);
  


//   sleep(15.0);

  

//   ros::shutdown();  
//   return 0;