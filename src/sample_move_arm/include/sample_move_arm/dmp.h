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

// struct ProjTrajectory{
// 	vector<double> points_dim;
// 	// vector<double> times;
// };
class Dmp{
public:
	Dmp();
	void Learning(const Trajectory &demo, const double &K, const double &D, const int &dimension);
	void Planning(const Point &start_state, const Point &goal_state, const double &tau, const double &dt, Trajectory &plan);
private:
	void Clear();
	double ComputePhase(double);
	void ComputeVelAcc();;
	void ComputeF();
	void InitializeVars();
	double LinearFunctionApproximator(double s);
	bool IsNearGoal(double);
	int dimension_;
	Trajectory demonstration_;
	double K_;
	double D_;
	double tau_;
	double alpha_;
	double start_phase_;
	double end_phase_;
	int num_points_;
	int num_iter_integr_;
	double x_start_; //demo
	double x_goal_; 	//demo
	vector<double> times_;
	vector<double> x_demo_;
	vector<double> v_demo_;
	vector<double> a_demo_;
	vector<double> f_phase_;
	vector<double> f_target_;

	double plan_start_;
	double plan_goal_;
	// vector< pair<double, double> > f_;

	double goal_threshold_;
};

class DmpGroup{
public:
	DmpGroup();
	void Learning(const Trajectory &demo, const double K, const double D);
	void Planning(const Point &start_state, const Point &goal_state, const double tau, const double dt, Trajectory &plan);
private:
	int n_dim_;
	Trajectory demo_;
	double K_;
	double D_;
	vector<Dmp*> all_dmp_;

};