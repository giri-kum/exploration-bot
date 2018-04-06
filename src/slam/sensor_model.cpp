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



//raycasting for detecting the expected distance from the pose to the nearest occupied cell in that direction
double SensorModel::raycast_dist(const particle_t& sample, const OccupancyGrid& map, float angle) {
    
    double z_est = z_max;

    
    Point<double> start_position = Point<double>(sample.pose.x, sample.pose.y);
    
    Point<int> curr_cell = global_position_to_grid_cell(start_position, map);
    Point<double> curr_pos = global_position_to_grid_position(start_position, map);

    //Point<double> ray_grid = grid_utils.global_position_to_grid_position( Point<double>(sample.x, sample.y), map);

    
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
    float th_global = angle + sample.pose.theta;
    /*
    
    // Determine quadrant (1-4)
    quad = 0;
    if (th_global < M_PI/4 && th_global >= -M_PI/4) quad = 1;
    else if (th_global < 3*M_PI/4 && th_global >= M_PI/4) quad = 2;
    else if (th_global >= 3*M_PI/4 || th_global < -3*M_PI/4) quad = 3;
    else if (th_global >= -3*M_PI/4 && th_global < -M_PI/4) quad = 4;
    //std::cout << "th_global: " << th_global << " quad: " << quad << "cell: " << las_cell.x << ", " << las_cell.y << std::endl;
    //if (quad == 0) std::cout << "quad is zero!" << std::endl;
    if (quad == 0) std::cout << "error! quad == 0" << std::endl;


    
    // Get properties of line
    deltax = las_cell.x - self_cell.x; //**should not be zero**
    deltay = las_cell.y - self_cell.y;
    deltaerr = fabs(deltay / deltax);



    // Boundary checking
    if (quad == 1 || quad == 3) {
        if (deltax == 0) deltaerr = 0;
        else deltaerr = fabs(deltay / deltax);
    }
    else if (quad == 2 || quad == 4) {
        if (deltay == 0) deltaerr = 0;
        else deltaerr = fabs(deltax / deltay);
    }

    // Update all cells along line with Bresenham's Line Algorithm
    x = self_cell.x; // start at self x
    y = self_cell.y;  // start at self y
    error = 0; // no error at start


    while (map.isCellInGrid(curr_cell.x, curr_cell.y)) {
        

        // Update error
        error += deltaerr;
        while (error >= 0.5) {
            if (quad == 1 || quad == 3) y += (deltay>0)?1:-1;
            else x += (deltax>0)?1:-1;
            error -= 1;
            //std::cout << "don't seg fault! error = " << error << std::endl;
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
        if ( map.operator()(x, y) > tip_val) {
            break;
        }

        //update cell
        curr_cell.x = 
        curr_cell.y = 


    }
    //end of copied code
    */

    Point<double> end_position  = grid_position_to_global_position( curr_pos, map);

    z_est = std::hypot(end_position.x - start_position.x, end_position.y - start_position.y);

    return 1.0; // DEBUG DEBUG DEBUG ***
    return z_est;
}

