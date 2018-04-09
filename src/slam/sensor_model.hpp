#ifndef SLAM_SENSOR_MODEL_HPP
#define SLAM_SENSOR_MODEL_HPP

#include <lcmtypes/lidar_t.hpp>

class  lidar_t;
class  OccupancyGrid;
struct particle_t;

/**
* SensorModel implement a sensor model for computing the likelihood that a laser scan was measured from a
* provided pose, give a map of the environment.
* 
* A sensor model is compute the unnormalized likelihood of a particle in the proposal distribution.
*
* To use the SensorModel, a single method exists:
*
*   - double likelihood(const particle_t& particle, const lidar_t& scan, const OccupancyGrid& map)
*
* likelihood() computes the likelihood of the provided particle, given the most recent laser scan and map estimate.
*/
class SensorModel
{
public:

    /**
    * Constructor for SensorModel.
    */
    SensorModel(void);

    /**
    * likelihood computes the likelihood of the provided particle, given the most recent laser scan and map estimate.
    * 
    * \param    particle            Particle for which the log-likelihood will be calculated
    * \param    scan                Laser scan to use for estimating log-likelihood
    * \param    map                 Current map of the environment
    * \return   Likelihood of the particle given the current map and laser scan.
    */
    double likelihood(const particle_t& particle, const lidar_t& scan, const OccupancyGrid& map);

private:
    
    ///////// TODO: Add any private members for your SensorModel ///////////////////

    //model paramaters

    //max sensor range
    float z_max;
    float z_hit;
    float z_short;
    float z_rand;
    float lambda_short;
    float sigma_hit;

    //sensor offsets from robot reference frame
    float x_sens;
    float y_sens;
    float theta_sens;

    //the value which we use to compare whether a cell is occupied or not
    float tip_val;

    //calculating the distance to obstacle using raycasting
    double raycast_dist(const particle_t& sample, const OccupancyGrid& map, float angle);

};

#endif // SLAM_SENSOR_MODEL_HPP
