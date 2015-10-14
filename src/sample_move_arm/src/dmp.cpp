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
#include <cmath>

using namespace std;
// typedef vector<Point>::iterator itpoint;

Dmp::Dmp()
{
	start_phase_ = 1;
	end_phase_ = 0.01;
	num_iter_integr_ = 10000;
}

double Dmp::ComputePhase(double t)
{
	double exponent = (-(alpha_ /tau_)*t);
	return exp(exponent);
}

void Dmp::ComputeVelAcc()
{
	v_start_ = 0;
	v_demo_.push_back(v_start_);

	a_start_ = 0;
	a_demo_.push_back(a_start_);

	for(int i =1; i<num_points_; i++)
	{
		double dx, dv, dt;
		dx = x_demo_[i]-x_demo_[i-1];
		dt = (times_[i]-times[i-1]);
		v_demo_.push_back((dx/dt));
		dv = v_demo_[i] - v_demo_[i-1];
		a_demo_.push_back((dv/dt));
	}

}
void Dmp::ComputeF()
{
	double phase, target;
	for(int i =0; i<num_points_; i++)
	{
		phase = ComputePhase(times_[i]);
		f_phase_.push_back(phase);
		target = (tau_*a_demo_[i]) + (D_*v_demo_[i]);
		target /= K_;
		target -= (x_goal_ - x_demo_[i]);
		target += (x_goal_ - x_start_)*phase;
		f_target_.push_back(target);
	}
}

void Dmp::InitializeVars()
{
	tau_ = demonstration_.times.back();
	times_ = demonstration_.times;
	alpha_ = (log(end_phase_)*(-1))*tau_;
	num_points_ = demonstration_.points.size();
	for(int i=0; i<num_points_; i++)
		x_demo_.push_back(demonstration_.points[i][dimension]);

	x_start_ = x_demo_.front();
	x_goal_ = x_demo_.back();	
}

void Dmp::Learning(const Trajectory &demo, double K, double D, int dimension)
{
	dimension_ = dimension;
	demonstration_ = *demo;
	K_ = K;
	D_ = D;
	InitializeVars();
	ComputeVelAcc();
	ComputeF();
}

void Dmp::Planning(Point &start_state, Point &goal_state, double tau, double time_res, Trajectory &plan)
{
	tau_ = tau;
	double t = 0;
	double phase;
	double dt = time_res/num_iter_integr_;
	double x = start_state.coordinates[dimension_];
	double v = start_state.velocities[dimension_] * tau;
	vector<double> traj_coords;
	vector<double> traj_vel;
	vector<double> times;
	while(t<tau)
	{	
		for(int i=0; i<num_iter_integr_; i++)
		{	
			phase = ComputePhase(t);
			f_s = LinearInterpolator(s);

			
		}
		traj_coords.push_back(x);
		traj_vel.push_back(v);
		times.push_back(t);

		t+=time_res;
	}

}

void Dmp::LinearInterpolator(double x)
{

}