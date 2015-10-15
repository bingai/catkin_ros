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
#include <fstream>
#include <cstdlib>
using namespace std;

Dmp::Dmp(bool policy)
{
	num_iter_integr_ = 10000;
	goal_threshold_ = 0.0001;

	//dont need to change ever
	start_phase_ = 1;
	end_phase_ = 0.01;
	policy_ = policy;
	Clear();
}

double Dmp::ComputePhase(double t)
{
	double exponent = (-(alpha_ /tau_)*t);
	return exp(exponent);
}

void Dmp::ComputeVelAcc()
{
	// ofstream ofile;
	// if(dimension_==0)
	// 	ofile.open("/home/rahul/git/catkin_ws/src/sample_move_arm/src/demovals0");
	// if(dimension_==1)
	// 	ofile.open("/home/rahul/git/catkin_ws/src/sample_move_arm/src/demovals1");
	// if(dimension_==2)
	// 	ofile.open("/home/rahul/git/catkin_ws/src/sample_move_arm/src/demovals2");
	// else
	// 	ofile.open("/home/rahul/git/catkin_ws/src/sample_move_arm/src/demoval");

	double v_start = 0;
	v_demo_.push_back(v_start);
	// a_start_ = 0;
	// a_demo_.push_back(a_start_);
	for(int i =1; i<num_points_; i++)
	{
		double dx, dv, dt, v_i, a_i;
		dx = x_demo_[i]-x_demo_[i-1];
		dt = (times_[i]-times_[i-1]);
		// v_demo_.push_back((dx/dt));
		
		if(i==num_points_-1)
			v_i = 0;
		else
			v_i = (2 * dx/dt) - (v_demo_[i-1]/tau_);
		
		v_demo_.push_back(v_i*tau_);
		dv = (v_demo_[i] - v_demo_[i-1])/tau_;
		a_demo_.push_back((dv/dt)); //for i-1
	}
	a_demo_.push_back(0);

	// if(dimension_==0)
	// {


	// for(int i=0; i<num_points_; i++)
	// {
	// 	cout<<x_demo_[i]<<" "<<v_demo_[i]<<" "<<a_demo_[i]<<endl;
	// }
	// ofile<<"Max element in a_demo is "<<*max_element(a_demo_.begin(), a_demo_.end())<<" ; min is "<<*min_element(a_demo_.begin(),a_demo_.end())<<endl;
	// ofile<<"Max element in v_demo is "<<*max_element(v_demo_.begin(), v_demo_.end())<<" ; min is "<<*min_element(v_demo_.begin(),v_demo_.end())<<endl;
	// ofile.close();
	//now v_demo and a_demo have size same
	// }
}
void Dmp::ComputeF()
{	
	// ofstream ofile;
	// if(dimension_==0)
	// 	ofile.open("/home/rahul/git/catkin_ws/src/sample_move_arm/src/f0");
	// if(dimension_==1)
	// 	ofile.open("/home/rahul/git/catkin_ws/src/sample_move_arm/src/f1");
	// if(dimension_==2)
	// 	ofile.open("/home/rahul/git/catkin_ws/src/sample_move_arm/src/f2");
	// else
	// 	ofile.open("/home/rahul/git/catkin_ws/src/sample_move_arm/src/f");

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
		// if(!dimension_)cout<<f_phase_[i]<<" "<<f_target_[i]<<endl;
		// pair<double, double> p = make_pair(phase, target);
		// f_.push_back(p);
	}
	// ofile.close();
}

void Dmp::SearchForParam(Point start_state, Point goal_state, double tau, double time_res)
{
	Trajectory t1;
	param_ = 0.5;
	Planning(start_state, goal_state, tau, time_res, t1);
	double min_cost = Cost(t1, goal_state);
	for(int i =0; i<10; i++)
	{	
		int randomNum = rand() % 19 + (-9);
		randomNum/=100; //to get number in range -0.09 to 0.09

		double p = param_ + randomNum;
		Trajectory t;
		Planning(start_state, goal_state, tau, time_res, t);
		if(Cost(t, goal_state)<min_cost)
		{
			param_ = p;
			min_cost = Cost(t, goal_state);
		}

		else
		{
			p = param_;
		}
	}
}

double Dmp::Cost(Trajectory &plan, Point &goal_state)
{

	double d=0;
	for (int i = 0 ; i< plan.points.front().coordinates.size(); i++)
	{
		d+=pow((plan.points.back().coordinates[i] - goal_state.coordinates[i]), 2);
	}
	d = sqrt(d);
	return d;
}

void Dmp::InitializeVars()
{
	tau_ = demonstration_.times.back();
	// if (!dimension_)
	// 	cout<<"Tau is "<<tau_<<endl;
	times_ = demonstration_.times;
	// cout<<"last time:"<<times_.back()<<endl;
	alpha_ = (log(end_phase_)*(-1));
	// cout<<"alpha:"<<alpha_<<endl;
	num_points_ = demonstration_.points.size();
	for(int i=0; i<num_points_; i++)
		x_demo_.push_back(demonstration_.points[i].coordinates[dimension_]);

	cout<<"Start and goal in dim="<<dimension_<<endl;
	x_start_ = x_demo_.front();
	x_goal_ = x_demo_.back();
	
	cout<<x_start_<<" ";
	cout<<x_goal_<<endl;


}

void Dmp::Learning(const Trajectory &demo, const double &K, const double &D, const int &dimension)
{
	Clear();

	dimension_ = dimension;
	demonstration_ = demo;
	K_ = K;
	D_ = D;
	InitializeVars();
	ComputeVelAcc();
	ComputeF();
	// cout<<"Learnt DMP for "<<dimension_<<endl;
	// cout<<"F_Phase_: ";
 //  	copy(f_phase_.begin(), f_phase_.end(), ostream_iterator<double>(cout, " "));
 //  	cout<<endl<<"F_target_: ";
 //  	copy(f_target_.begin(), f_target_.end(), ostream_iterator<double>(cout, " "));
 //  	cout<<endl<<"----------------------------------------------"<<endl;
}

void Dmp::Clear()
{
	x_demo_.clear();
	v_demo_.clear();
	a_demo_.clear();
	times_.clear();
	f_phase_.clear();
	f_target_.clear();

}

void Dmp::Planning(const Point &start_state, const Point &goal_state, const double &tau, const double &time_res, Trajectory &plan)
{
	tau_ = tau;
	// cout<<"In lower planning"<<endl;
	double t = 0;
	double dt = time_res/num_iter_integr_;
	
	plan_start_ = start_state.coordinates[dimension_];
	plan_goal_ = goal_state.coordinates[dimension_];

	//for similarity
	// double x_start = plan_start_;
	// double v_start = start_state.velocities[dimension_];
	//check segfault
	// double v_start = 0;

	vector<double> traj_coords;
	// vector<double> traj_vel;
	vector<double> times;
	vector<Point> traj_points;



	if(policy_)
	{
		SearchForParam(start_state, goal_state, tau, time_res);
	}
	//for inside loop
	double phase, f_s;
	double x = plan_start_;
	double v = 0;
	while(t<tau)// && !IsNearGoal(x))
	{	
		for(int i=0; i<num_iter_integr_; i++)
		{	

			phase = ComputePhase(t+(dt*i));
			if(!policy_)
				f_s = LinearFunctionApproximator(phase);
			else
				f_s = ParamLinearFunctionApproximator(phase);
			// cout<<f_s<<endl;
			double v_dot = K_*(plan_goal_ - x) - D_*v;
			v_dot -= K_*(plan_goal_ - plan_start_)*phase;
			v_dot += K_*f_s;
			v_dot /= tau;

			double x_dot = v/tau;
			
			x += x_dot*dt;
			v += v_dot*dt;
		}
		t+=time_res;
		// cout<<t<<" "<<x<<" "<<v<<endl;
		traj_coords.push_back(x);
		// traj_vel.push_back(v);
		// cout<<"1"<<endl;
		Point p;
		p.coordinates.push_back(x);
		// cout<<x<<" ";
		// cout<<"2"<<endl;
		times.push_back(t);
		// cout<<"3"<<endl;
		plan.points.push_back(p);
		// cout<<"4"<<endl;
	}
	// cout<<endl;
	plan.times = times;
	// cout<<"5"<<endl;
	// cout<<"Dim "<<dimension_<<" planned path with length "<<traj_coords.size()<<" and final coord is "<<traj_coords.back()<<". Time is "<<t<<endl;


}

double Dmp::LinearFunctionApproximator(double s)
{	
	if(s>1.0 || s<=0.0)
		return 0.0;
	
	int j;

	int i=0;
	while(f_phase_[i]>s && i<num_points_)
	{
		i++;
	}
	//new i is lower than s or equal to s

	if(f_phase_[i]==s)
		return f_target_[i];

	if(s>f_phase_[0]) //larger than any demo phase //extrapolation in this case
	{
		return (f_target_[0]/f_phase_[0] )* (1.0 - s);

	}

	else if(i==num_points_-1)
	{
		return (f_target_[num_points_-1]/f_phase_[num_points_-1])*s;

	}
		
	else
	{
		j = i-1;
		//at i element is smaller. i-1 is larger
		//
		double slope = (f_target_[i] - f_target_[j])/(f_phase_[i] - f_phase_[j]);
		return f_target_[i] + slope* (s-f_phase_[i]);
	}
}

double Dmp::ParamLinearFunctionApproximator(double s)
{	
	//param in range 0-1
	if(s>1.0 || s<=0.0)
		return 0.0;
	
	int j;

	int i=0;
	while(f_phase_[i]>s && i<num_points_)
	{
		i++;
	}
	//new i is lower than s or equal to s

	if(f_phase_[i]==s)
		return f_target_[i];

	if(s>f_phase_[0]) //larger than any demo phase //extrapolation in this case
	{
		return param_*((f_target_[0]/f_phase_[0] )* (1.0 - s));

	}

	else if(i==num_points_-1)
	{
		return param_*((f_target_[num_points_-1]/f_phase_[num_points_-1])*s);

	}
		
	else
	{
		j = i-1;
		//at i element is smaller. i-1 is larger
		//
		double slope = (f_target_[i] - f_target_[j])/(f_phase_[i] - f_phase_[j]);
		return f_target_[i] + slope*(param_*(s-f_phase_[i]));
	}
}
DmpGroup::DmpGroup(bool policy){
	policy_ = policy;
}

void DmpGroup::Learning(const Trajectory &demo, const double K, const double D)
{
	K_ = K;
	D_ = D;
	demo_ = demo;
	n_dim_ = demo.points.front().coordinates.size();
	for(int i = 0 ; i<n_dim_; i++)
	{
	  Dmp* d = new Dmp(policy_);
	  d->Learning(demo_, K_, D_, i);
	  all_dmp_.push_back(d);
	}
	cout<<"Learnt all "<<all_dmp_.size()<<" DMPs"<<endl;

}

void DmpGroup::Planning(const Point &start_state, const Point &goal_state, const double tau, const double time_res, Trajectory &final_plan)
{
    cout<<"Planning start state is ";
	for(int p = 0; p<start_state.coordinates.size();p++)
		cout<<p<<":"<<start_state.coordinates[p]<<",";
	cout<<endl;

	cout<<"Planning goal state is ";
	for(int p = 0; p<goal_state.coordinates.size();p++)
		cout<<p<<":"<<goal_state.coordinates[p]<<",";
	cout<<endl;
    

	// cout<<"Planning started"<<endl;
	for(int i =0; i<n_dim_; i++)
	{
		Trajectory projection_traj;
		all_dmp_[i]->Planning(start_state, goal_state, tau, time_res, projection_traj);
		cout<<"Planned "<<i<<endl;
		// if (!i)
		// 	final_plan.times = projection_traj.times;

		int c = 0;
		for(int j=0; j<projection_traj.points.size(); j++)
		{
			// cout<<"Point "<<c<<": ";
			if(!i)
			{
				Point p;
				p.coordinates.push_back(projection_traj.points[j].coordinates[0]);
				final_plan.points.push_back(p);
			}
			else
			{
				final_plan.points[j].coordinates.push_back(projection_traj.points[j].coordinates[0]);
			}
			c++;
		}
	}
	cout<<"Final plan is of length "<<final_plan.points.size()<<endl;
}


// bool Dmp::IsNearGoal(double x)
// {
// 	if(abs(x-plan_goal_)<goal_threshold_)
// 		return true;
// 	else
// 		return false;
// }