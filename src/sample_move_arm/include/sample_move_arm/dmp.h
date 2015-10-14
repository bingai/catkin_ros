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

struct Point{
  vector<double> coordinates;
  vector<double> velocities;
};

struct Trajectory{
  vector< Point > points;
  vector< double > times;
};

class Dmp{
public:
	Dmp(double K, double D);
	void Learning(const Trajectory &demo, double K, double D, int dimension);
	void Planning(Point &start_state, Point &goal_state, double tau, double dt, Trajectory &plan);
private:
	void ComputePhase(double);
	void ComputeVelAcc();;
	void ComputeF();
	void InitializeVars();
	int dimension_;
	Trajectory demonstration_;
	double K_;
	double D_;
	double tau_;
	double alpha_;
	int num_points_;
	int num_iter_integr_;
	double x_start_;
	double x_goal_; 
	vector<double> times_;
	vector<double> x_demo_;
	vector<double> v_demo_;
	vector<double> a_demo_;
	vector<double> f_phase_;
	vector<double> f_target_;
};
