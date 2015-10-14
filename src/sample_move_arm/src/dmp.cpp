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
#include <algorithm>

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
	double v_start_ = 0;
	v_demo_.push_back(v_start_);

	// a_start_ = 0;
	// a_demo_.push_back(a_start_);

	for(int i =1; i<num_points_; i++)
	{
		double dx, dv, dt, v_i, a_i;
		dx = x_demo_[i]-x_demo_[i-1];
		dt = (times_[i]-times_[i-1]);
		// v_demo_.push_back((dx/dt));
		v_i = v_demo_[i-1] + (2 * dx/dt);
		v_demo_.push_back(v_i);

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
		pair<double, double> p = make_pair(phase, target);
		f_.push_back(p);
	}
}

void Dmp::InitializeVars()
{
	tau_ = demonstration_.times.back();
	times_ = demonstration_.times;
	alpha_ = (log(end_phase_)*(-1))*tau_;
	num_points_ = demonstration_.points.size();
	for(int i=0; i<num_points_; i++)
		x_demo_.push_back(demonstration_.points[i].coordinates[dimension_]);

	x_start_ = x_demo_.front();
	x_goal_ = x_demo_.back();	
}

void Dmp::Learning(const Trajectory &demo, double K, double D, int dimension)
{
	dimension_ = dimension;
	demonstration_ = demo;
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
	double dt = time_res/num_iter_integr_;
	double x_start = start_state.coordinates[dimension_];
	double v_start = start_state.velocities[dimension_];
	double x_goal = goal_state.coordinates[dimension_];
	//check segfault
	vector<double> traj_coords;
	vector<double> traj_vel;
	vector<double> times;
	vector<Point> traj_points;


	double phase, f_s;
	double x = x_start;
	double v = v_start;
	while(t<tau)
	{	

		for(int i=0; i<num_iter_integr_; i++)
		{	
			phase = ComputePhase(t);
			f_s = LinearFunctionApproximator(phase);

			double v_dot = K_*(x_goal - x) - D_*v;
			v_dot -= K_*(x_goal - x_start)*phase;
			v_dot += K_*f_s;
			v_dot /= tau;

			double x_dot = v/tau;
			
			x += x_dot*dt;
			v += v_dot*dt;
		}
		traj_coords.push_back(x);
		// traj_vel.push_back(v);
		
		Point p;
		// p.velocities.push_back(v);
		p.coordinates.push_back(x);
		
		t+=time_res;
		// times.push_back(t);
		plan.points.push_back(p);
	}

	plan.times = times;

}

double Dmp::LinearFunctionApproximator(double s)
{	
	if(s>1.0 || s<0.0)
		return 0.0;
	
	int j;

	int i=0;
	while(f_phase_[i]>s && i<num_points_)
		i++;
	
	if(i==0) //larger than any demo phase //extrapolation in this case
		j = i+1;

	else if(i==num_points_-1)
		j = num_points_-1;
		
	else
		j = i-1;

		//at i element is smaller. i-1 is larger
		//
		double slope = (f_target_[i] - f_target_[j])/(f_phase_[i] - f_phase_[j]);
		return f_target_[i] + slope* (s-f_phase_[i]);
}

DmpGroup::DmpGroup()
{

}

void DmpGroup::Learning(const Trajectory &demo, double K, double D)
{
	K_ = K;
	D_ = D;
	demo_ = demo;
	n_dim_ = demo.points.front().coordinates.size();
	vector<Trajectory> projections;
	for(int i = 0 ; i<n_dim_; i++)
	{
	  Dmp* d = new Dmp();
	  d->Learning(demo_, K_, D_, i);
	  all_dmp_.push_back(d);
	}
}
void DmpGroup::Planning(Point &start_state, Point &goal_state, double tau, double time_res, Trajectory &final_plan)
{
	for(int i =0; i<n_dim_; i++)
	{
		Trajectory projection_traj;
		all_dmp_[i]->Planning(start_state, goal_state, tau, time_res, projection_traj);

		// if (!i)
		// 	final_plan.times = projection_traj.times;

		for(int j = 0; j<projection_traj.points.size(); j++)
		{
			if(!j)
				final_plan.points.push_back(projection_traj.points[j]);
			
			else
			{
				final_plan.points[j].coordinates.push_back(projection_traj.points[j].coordinates[0]);
				// final_plan.points[j].velocities.push_back(projection_traj.points[j].velocities[0]);
			}
		}
	}
}

