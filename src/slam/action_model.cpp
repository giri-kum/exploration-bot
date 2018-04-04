#include <slam/action_model.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <iostream>
#include <random>

using namespace std;

default_random_engine generator; //must to seeded only once, so must be global.

double sample_gaussian(double variance,double mean)
{
	double stddev = sqrt(variance);
  	normal_distribution<double> distribution(mean,stddev);
  	return distribution(generator);
}


ActionModel::ActionModel(void)
{
	int factor = 100;
	float beta[4] = {0,0,0,0};//{(float)0.05/factor,(float)0.001/factor, (float)0.05/factor, (float)0.01/factor}; 
    //////////////// TODO: Handle any initialization for your ActionModel /////////////////////////
	oldpose.x = 0;
	oldpose.y = 0;
	oldpose.theta = 0;
	oldpose.utime = 0;

	del_rot1 = 0;
	del_trans = 0;
	del_rot2 = 0;
	time_stamp = 0;


	alpha[0] = beta[0]*beta[0];
	alpha[1] = beta[1]*beta[1];
	alpha[2] = beta[2]*beta[2];
	alpha[3] = beta[3]*beta[3];
}


bool ActionModel::updateAction(const pose_xyt_t& odometry)
{
    ////////////// TODO: Implement code here to compute a new distribution of the motion of the robot ////////////////

	double threshold[3] = {0.0001,0.0001,0.0001}; //0.1 mm and 0.0001 radians which 0.00572958 degrees
	double change_in_pose[3]; 
	change_in_pose[0] = odometry.x - oldpose.x;
	change_in_pose[1] = odometry.y - oldpose.y;
	change_in_pose[2] = odometry.theta - oldpose.theta;
	if(fabs(change_in_pose[0])>threshold[0] || fabs(change_in_pose[1]) > threshold[1] || fabs(change_in_pose[2]) > threshold[2])
	{
		std::cout<<"Robot moved: "<<change_in_pose[0]<<" "<<change_in_pose[1]<<std::endl;
	    del_rot1 = atan2(change_in_pose[1],change_in_pose[0]) - oldpose.theta;
	    del_trans = sqrt(change_in_pose[0]*change_in_pose[0] + change_in_pose[1]*change_in_pose[1]);
	    del_rot2 =  change_in_pose[2] - del_rot1;
	    time_stamp = odometry.utime;
	    oldpose = odometry; //Error?
	    moved = true;
	}
	else
	{
		moved =  false;
		std::cout<<"Robot unmoved: "<<change_in_pose[0]<<" "<<change_in_pose[1]<<std::endl;
	}
    return moved;
}


particle_t ActionModel::applyAction(const particle_t& sample)
{
	double del_bar_rot1, del_bar_trans, del_bar_rot2;
	particle_t new_particle;
    ////////////// TODO: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.
	if(moved)
	{
	del_bar_rot1 = del_rot1 - sample_gaussian(alpha[0]*del_rot1*del_rot1 + alpha[1]*del_trans*del_trans); // Alphas need to be tuned
	del_bar_trans = del_trans - sample_gaussian(alpha[2]*del_trans*del_trans + alpha[3]*(del_rot1*del_rot1 + del_rot2*del_rot2));
	del_bar_rot2 = del_rot2 -sample_gaussian(alpha[0]*del_rot2*del_rot2 + alpha[1]*del_trans*del_trans);
	new_particle.pose.x = sample.parent_pose.x + del_bar_trans*cos(sample.parent_pose.theta + del_bar_rot1);
	new_particle.pose.y = sample.parent_pose.y + del_bar_trans*sin(sample.parent_pose.theta + del_bar_rot1);
	new_particle.pose.theta = sample.parent_pose.theta + del_bar_rot1 + del_bar_rot2; // Don't forget to wrap the angle
	new_particle.pose.utime = time_stamp; // what time stamp should be given here
	new_particle.parent_pose = sample.pose;	
	}
		
    return new_particle;
}


