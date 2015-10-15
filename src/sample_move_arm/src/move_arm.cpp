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
#include <algorithm> // for copy
#include <iterator> // for ostream_iterator
#include <fstream>
// const double tau = 5.0;
using namespace std;
vector<geometry_msgs::PoseStamped> demoPoseStamped;

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
  // cout<<" I heard "<<ar.poses[0].pose.position.x<<" "<<ar.poses[0].pose.position.y<<" "<<ar.poses[0].pose.position.z<<" "<<ar.poses[0].header.stamp<<endl;
}

// void receiveTestArray(geometry_msgs::PoseArray ar)
// {
//   if(!test_assigned)
//   { 
//     for(int i=0; i<ar.poses.size(); i++)
//       testPoses.push_back(ar.poses[i]);

//     test_assigned = true;
//   }
//   // cout<<" I heard "<<ar.poses[0].position.x<<" "<<ar.poses[0].position.y<<" "<<ar.poses[0].position.z<<endl;
//   // cout<<" I heard "<<ar.poses[1].orientation.x<<" "<<ar.poses[1].orientation.y<<" "<<ar.poses[1].orientation.z<<endl;
// }

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
      // pt.coordinates.push_back(p.orientation.x);
      // pt.coordinates.push_back(p.orientation.y);
      // pt.coordinates.push_back(p.orientation.z);
      // pt.coordinates.push_back(p.orientation.w);
      demo.points.push_back(pt);
      demo.times.push_back(((*it).header.stamp.toSec() - start_time));
    }
    cout<<"Converted data to trajectory with "<<demo.times.size()<<" timesteps ";
    cout <<"and "<<demo.points.size()<<" points "<<endl;
    cout<<"One point is like :";
    copy(demo.points.front().coordinates.begin(), demo.points.front().coordinates.end(), ostream_iterator<double>(cout, " "));
    cout<<endl;
}

void ConvertTrajectoryToVecPoses(Trajectory &traj, vector<geometry_msgs::Pose> &poses)
{

  ofstream planout;
  planout.open("/home/rahul/git/catkin_ws/src/sample_move_arm/out/planposes.txt");
  for(int i=0; i<traj.points.size();i++)
  {
    geometry_msgs::Pose p;
    p.position.x = traj.points[i].coordinates[0];
    planout<<p.position.x<<" ";
    p.position.y = traj.points[i].coordinates[1];
    planout<<p.position.y<<" ";
    p.position.z = traj.points[i].coordinates[2];
    planout<<p.position.z<<endl;
    // p.orientation.x = traj.points[i].coordinates[3];
    // planout<<p.orientation.x<<" ";
    // p.orientation.y = traj.points[i].coordinates[4];
    // planout<<p.orientation.y<<" ";
    // p.orientation.z = traj.points[i].coordinates[5];
    // planout<<p.orientation.z<<" ";
    // p.orientation.w = traj.points[i].coordinates[6];
    // planout<<p.orientation.w<<endl;
    poses.push_back(p);
  }
  planout.close();
  cout<<"Final trajectory has "<<traj.points.size()<<"points. Start point is like ";
  copy(traj.points.front().coordinates.begin(), traj.points.front().coordinates.end(), ostream_iterator<double>(cout, " "));
  cout<<endl<<"End point is like: ";
  copy(traj.points.back().coordinates.begin(), traj.points.back().coordinates.end(), ostream_iterator<double>(cout, " "));

} 

void ConvertPosesToPoints(Point &start, Point &goal, geometry_msgs::Pose st, geometry_msgs::Pose go)
{
    start.coordinates.push_back(st.position.y);
    start.coordinates.push_back(st.position.x);
    start.coordinates.push_back(st.position.z);
    // start.coordinates.push_back(st.orientation.x);
    // start.coordinates.push_back(st.orientation.y);
    // start.coordinates.push_back(st.orientation.z);
    // start.coordinates.push_back(st.orientation.w);
    goal.coordinates.push_back(go.position.y);
    goal.coordinates.push_back(go.position.x);
    goal.coordinates.push_back(go.position.z);
    // goal.coordinates.push_back(go.orientation.x);
    // goal.coordinates.push_back(go.orientation.y);
    // goal.coordinates.push_back(go.orientation.z);
    // goal.coordinates.push_back(go.orientation.w);
  
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

  // ros::Subscriber sub2 = node_handle.subscribe("pub_teststates", 1000, receiveTestArray);
  // while(!test_assigned)
  // {
  //   sleep(1);
  // }


  Trajectory demonstration, dmp_plan;
  ConvertDataToTrajectory(demonstration);
  // Point start, goal;
  // ConvertPosesToPoints(start, goal);
  
  // start = demonstration.points.front();
  // copy(start.coordinates.begin(), start.coordinates.end(), ostream_iterator<double>(cout, " "));
  // cout<<endl;
  // goal = demonstration.points.back();
  // copy(goal.coordinates.begin(), goal.coordinates.end(), ostream_iterator<double>(cout, " "));

  DmpGroup d(true);

  double K=pow(10,8);
  double D = sqrt(K)*2;
  d.Learning(demonstration, K, D);

  robot_state::RobotState start_state(*group.getCurrentState());
  // start_state.printStateInfo(std::cout);
  geometry_msgs::Pose start_pose2;
  // start_pose2.orientation.w = 1.0;
  start_pose2.position.x = 0.01;
  start_pose2.position.y = 0.2;
  start_pose2.position.z = 0.35;

  const robot_state::JointModelGroup *joint_model_group =
                  start_state.getJointModelGroup(group.getName());
  start_state.setFromIK(joint_model_group, start_pose2);
  group.setStartState(start_state);

  geometry_msgs::PoseStamped st =  group.getCurrentPose();
  
  geometry_msgs::Pose goal_pose;
  goal_pose.position.x = 0.0001;
  goal_pose.position.y = 0.01;
  goal_pose.position.z = 0.02;
  // goal_pose.orientation.x = 0;
  // goal_pose.orientation.y = 0;
  // goal_pose.orientation.z = 0;
  // goal_pose.orientation.w = 0;

  Point start, goal;
  ConvertPosesToPoints(start, goal, start_pose2, goal_pose);

  d.Planning(start, goal, 4, 0.01, dmp_plan);

  vector<geometry_msgs::Pose> poses;
  ConvertTrajectoryToVecPoses(dmp_plan, poses);

  moveit_msgs::RobotTrajectory trajectory;
  double fraction = group.computeCartesianPath(poses,
                                               0.01,  // eef_step
                                               0.0,   // jump_threshold
                                               trajectory);

  ROS_INFO("Executing trajectory (cartesian path)");

  moveit::planning_interface::MoveGroup::Plan plan;
  plan.trajectory_ = trajectory;
  group.execute(plan);



  sleep(15.0);
  ros::shutdown();  
  return 0;
}
  // PlotPlan(poses, node_handle);

  // cout<<"Path planned has start: "<<poses.front().position.x<<" "<<poses.front().position.y<<" "<<poses.front().position.z<<endl;
  // cout<<"Path planned has goal: "<<poses.back().position.x<<" "<<poses.back().position.y<<" "<<poses.back().position.z<<endl;


