#include <slam/sensor_model.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/grid_utils.hpp>
#include <random>


//for generating gaussian distributions
static std::default_random_engine generator; //make sure this is declared only once

double sample(double mean, double variance) {
	double stddev = std::sqrt(variance);
	std::normal_distribution<double> distribution(mean,stddev);
	return distribution(generator);
    }


//raycasting for detecting the expected distance from the pose to the nearest occupied cell in that direction
double SensorModel::raycast_dist(const particle_t& sample, const OccupancyGrid& map, float angle) {
    
	double z_est = z_max;

    
	Point<double> start_position = Point<double>(sample.pose.x, sample.pose.y);
    
	Point<int> curr_cell = global_position_to_grid_cell(start_position, map);
	Point<double> curr_pos = global_position_to_grid_position(start_position, map);

	//Point<double> ray_grid = grid_utils.global_position_to_grid_position( Point<double>(sample.x, sample.y), map);

    /*
	//Copied from mapping.cpp

	float deltax;
    float deltay;
    float deltaerr;
    float error;
    int x;
    int y;
    int quad;
    int th_temp;
    float self_theta;
    float th_global;

    
    // Find quadrant (1-4)
    th_temp = -M_PI/4;
    quad = 1;
    while (th_global - th_temp > PI/2) {
        quad_temp += PI/2;
        quad += 1;
    }
    // Get properties of line
    deltax = las_cell.x - self_cell.x; //**should not be zero**
    deltay = las_cell.y - self_cell.y;
    deltaerr = fabs(deltay / deltax);
    // Update all cells along line with Bresenham's Line Algorithm
    x = self_cell.x; // start at self x
    y = self_cell.y;  // start at self y
    error = 0; // no error at start
    while (true) {
        
        // Update error
        error += deltaerr;
        while (error >= 0.5) {
            if (quad == 1 || quad == 3) y += sign(deltay);
            else x += sign(deltax);
            error -= 1;
        }

        // Increment and check if done based on quadrant
        if (quad == 1) {
            x++;
        }
        else if (quad == 2) {
            y++;
        }
        else if (quad == 3) {
            x--;
        }
        else if (quad == 4) {
            y--;
        }

        //if the cell is occupied break
        if ( map(x, y) > tip_val) {
        	break;
        }

        //if cell is out of grid break
        if( !map.isCellInGrid(curr_cell.x, curr_cell.y) ) {
			break;
		}

    }
	//end of copied code
    */

    Point<double> end_position  = grid_position_to_global_position( curr_pos, map);

    z_est = std::hypot(end_position.x - start_position.x, end_position.y - start_position.y);

    return 1.0; // DEBUG DEBUG DEBUG ***
    return z_est;
}



SensorModel::SensorModel(void)
{
    ///////// TODO: Handle any initialization needed for your sensor model
    z_hit = 0;
    z_max = 100;
    z_short = 0;
    z_rand = 0;

    lambda_short = 0;
    sigma_hit = 0;

    x_sens = 0;
    y_sens = 0;
    theta_sens = 0;

    tip_val = 0;
}


double SensorModel::likelihood(const particle_t& sample, const lidar_t& scan, const OccupancyGrid& map)
{
    ///////////// TODO: Implement your sensor model for calculating the likelihood of a particle given a laser scan //////////

    /*Pseudocode for beam range finding model
    q=1
    for all measurements m in scan do
    	use ray casting to cunpute noise free range for given location and direction
    	determine probability p of each individual measurement
    	q=q*p
    return q
    */

	//probability of this point being the right one
    double q = 1;
    
    //iterate through all of the measurements in the scan
    for (int32_t k = 0; k < scan.num_ranges; k++) {

    	//probabilities of for actual hit, max range, short, or random noise
    	double p_hit = 0;
    	double p_max = 0;
    	double p_short = 0;
    	double p_rand = 0;

    	//calculate z_tk*
    	double z_est = raycast_dist(sample, map, scan.thetas[k]);

    	//p = zhit * phit(ztk | xt, m) + zshort * pshort(ztk | xt, m) + zmax * pmax(ztk | xt, m) + zrand * prand(ztk | xt, m)

    	if ( (0 <= scan.ranges[k]) && (scan.ranges[k] <= z_max)) {
    		float hit_dist = 1 / std::sqrt(2 * M_PI * std::pow(sigma_hit, 2)  ) * std::exp(-1/2 * std::pow((scan.ranges[k] - z_est), 2) / std::pow(sigma_hit, 2) );
    		float hit_normalizer = 1 / std::sqrt(2 * M_PI * std::pow(sigma_hit, 2) ) * ( (-1.25331 * sigma_hit * std::erf(.707107 * (z_est - z_max) / sigma_hit )) - (-1.25331 * sigma_hit * std::erf(.707107 * (z_est) / sigma_hit )) );
    		p_hit = hit_normalizer * hit_dist;
    	}

    	if ( (0 <= scan.ranges[k]) && (scan.ranges[k] <= z_est) ) {
    		p_short = 1/(1-std::exp(-lambda_short * z_est)) * lambda_short * std::exp(-lambda_short * z_est);
    	}

    	if (scan.ranges[k] >= z_max) {
    		p_max = 1;
    	}

    	if (scan.ranges[k] < z_max) {
    		p_rand = 1 / z_max;
    	}

    	q = q * (z_hit * p_hit + z_max * p_max + z_short * p_short + z_rand * p_rand);

	}
    
    return q;
}

