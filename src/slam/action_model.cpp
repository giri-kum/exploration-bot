#include <slam/action_model.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <iostream>
#include <random>
//#include <typeinfo>
using namespace std;

default_random_engine generator; //must to seeded only once, so must be global.

double sample_gaussian(double variance,double mean)
{
	double stddev = sqrt(variance);
  	normal_distribution<double> distribution(mean,stddev);
  	return distribution(generator);
}

double wrap_angle(double given_angle)
{
	double result = given_angle;
	while(result < - M_PI)
		result = result + 2*M_PI;
	while(result >= M_PI)
		result = result - 2*M_PI;
	return result;
}

ActionModel::ActionModel(void)
{
	//float a1_rot = 0.005; // rad; backup_value = 0.0005; set {0,sqrt(0.03), sqrt(0.0025)}
	//float a2_rot_from_trans = 0.01; // rad/m; backup_value = 0.001 or 0.0001; set = {0.125,sqrt(30),sqrt(0.0001)}
	//float a3_trans = 0.5; // m; backup_value = 0.05; set = {0.1,1,sqrt(0.5)}
	//float a4_trans_from_rot = 0.01; // m/rad; backup_value = 0.001 or 0.0001; set = {0,sqrt(0.0015),sqrt(0.0005)}
	
	/* Peter's values
	alpha[0] = 0.01*0.01;//beta[0]*beta[0];
	alpha[1] = 0.0001*0.0001;//beta[1]*beta[1];
	alpha[2] = 0.0025*0.0025;//beta[2]*beta[2];
	alpha[3] = 0.0001*0.0001;//beta[3]*beta[3];
	*/
	/*float a1_rot = 0.001; // rad; backup_value = 0.0005; set {0,sqrt(0.03), sqrt(0.0025)}
	float a2_rot_from_trans = 0.0001; // rad/m; backup_value = 0.001 or 0.0001; set = {0.125,sqrt(30),sqrt(0.0001)}
	float a3_trans = 0.0025; // m; backup_value = 0.05; set = {0.1,1,sqrt(0.5)}
	float a4_trans_from_rot = 0.001;*/

	float a1_rot = 0.1; //0.1 rad; 0.15 for log 
	float a2_rot_from_trans = 0.5; //0.5 rad/m; 
	float a3_trans = sqrt(0.0025);//sqrt(0.0025); // m; 
	float a4_trans_from_rot = 0.0001; //0.0001
	

	float beta[4] = {a1_rot, a2_rot_from_trans, a3_trans, a4_trans_from_rot};  //for debugging {(float)0.0005/factor,(float)0.001/factor, (float)0.05/factor, (float)0.001/factor}; 
    
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
	    del_rot1 = atan2(change_in_pose[1],change_in_pose[0]) - oldpose.theta;
	    del_trans = sqrt(change_in_pose[0]*change_in_pose[0] + change_in_pose[1]*change_in_pose[1]);
	    del_rot2 =  change_in_pose[2] - del_rot1;
	    if(del_trans > 0.001)
	    	del_rot1 = del_rot1;
	    else
	    	del_rot1 = del_rot2;
	    time_stamp = odometry.utime;
	    oldpose = odometry; //Error?
	    moved = true;
	}
	else
	{
		moved =  false;
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
	new_particle.parent_pose = sample.pose;	
	del_bar_rot1 = del_rot1 + sample_gaussian(alpha[0]*del_rot1*del_rot1 + alpha[1]*del_trans*del_trans,0); // Alphas need to be tuned
	del_bar_trans = del_trans + sample_gaussian(alpha[2]*del_trans*del_trans + alpha[3]*(del_rot1*del_rot1 + del_rot2*del_rot2),0);
	del_bar_rot2 = del_rot2 +sample_gaussian(alpha[0]*del_rot2*del_rot2 + alpha[1]*del_trans*del_trans,0);
	new_particle.pose.x = sample.pose.x + del_bar_trans*cos(sample.pose.theta + del_bar_rot1);
	new_particle.pose.y = sample.pose.y + del_bar_trans*sin(sample.pose.theta + del_bar_rot1);
	//std::cout << "thetas " << del_bar_rot1 - del_rot1 << " " << del_bar_rot2 - del_rot2 << std::endl;
	new_particle.pose.theta = sample.pose.theta + del_bar_rot1 + del_bar_rot2; // Don't forget to wrap the angle

	new_particle.pose.utime = time_stamp; // what time stamp should be given here
	}
		
    return new_particle;
}


