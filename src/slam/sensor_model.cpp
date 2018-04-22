#include <slam/sensor_model.hpp>
//#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/grid_utils.hpp>
#include <slam/moving_laser_scan.hpp>
#include <random>
#include <common/angle_functions.hpp>


//for generating gaussian distributions, unused
/*
static std::default_random_engine generator; //make sure this is declared only once

double sample(double mean, double variance) {
	double stddev = std::sqrt(variance);
	std::normal_distribution<double> distribution(mean,stddev);
	return distribution(generator);
    }
*/

static std::random_device rd;
static std::mt19937_64 rng(rd());


SensorModel::SensorModel(void)
{
    //Initialization for sensor model

    //seems to dominate, in relaity probaly more like .95
    z_hit = .95;


    //pulled from data sheet
    z_max = 40;

    z_short = 0.0; //0.0018;
    z_rand = 0.0; //00000001;

    /*
    //both these seem to be very small
    z_short = .0018;
    z_rand = 0;
    */

    //arbitrarily small
    lambda_short = .000000001;

    //based on data
    //sigma_hit = .0022;

    //sigma_hit = .1;
    sigma_hit = .1;

    x_sens = 0;
    y_sens = 0;
    theta_sens = 0;

    tip_val = 0;

    n_samples = 20;
}


double SensorModel::likelihood(const particle_t& sample, const lidar_t& scan, const OccupancyGrid& map)
{
    //Sensor model for calculating the likelihood of a particle given a laser scan

    /*Pseudocode for beam range finding model
    q=1
    for all measurements m in scan do
    	use ray casting to compute noise free range for given location and direction
    	determine probability p of each individual measurement
    	q=q*p
    return q
    */

    // Interpolation
    float r_las;                // distance from self to laser scan termination (m)
    float th_global;
    MovingLaserScan moveLaser(scan, sample.parent_pose, sample.pose);
    adjusted_ray_t adjLaser;

	//probability of this point being the right one
    double q = 1;
    
    //iterate through all of the measurements in the scan
    for (int32_t k = 0; k < scan.num_ranges; k+=(int)(scan.num_ranges/n_samples) ) {

        //print out for calibration
        //std::cout << "theta " << scan.thetas[k] << " dist: " << scan.ranges[k] << std::endl;


        //If we are not using the adjusted laser, use this
        r_las = scan.ranges[k];
        th_global = -scan.thetas[k] + sample.pose.theta + theta_sens;


        // If we are using adjusted laser, use this
        //Extract laser data
        adjLaser = moveLaser[k];
        r_las = adjLaser.range;
        th_global = adjLaser.theta;

    	//probabilities of for actual hit, max range, short, or random noise
    	double p_hit = 0;
    	double p_max = 0;
    	double p_short = 0;
    	double p_rand = 0;

    	//calculate z_tk*
    	//double z_est = raycast_dist(sample, map, scan.thetas[k]);
        double z_est = raycast_dist(sample, map, th_global);
        //std::cout << "k      " << k << "   z_est:   " << z_est << "   z_meas   " << r_las << "     q" << q << std::endl;
        //double simp_z_est = simple_raycast_dist(sample, map, scan.thetas[k]);

    	//p = zhit * phit(ztk | xt, m) + zshort * pshort(ztk | xt, m) + zmax * pmax(ztk | xt, m) + zrand * prand(ztk | xt, m)

    	if ( (0 <= r_las) && (r_las <= z_max)) {
    		float hit_dist = 1.0 / (std::sqrt(2.0 * M_PI * std::pow(sigma_hit, 2)  )) * std::exp(-1.0/2.0 * std::pow((r_las - z_est), 2) / std::pow(sigma_hit, 2) );
    		float hit_normalizer = 1.0 / ((std::sqrt(2.0 * M_PI * std::pow(sigma_hit, 2) )) * ( (-1.25331 * sigma_hit * std::erf(.707107 * (z_est - r_las) / sigma_hit )) - (-1.25331 * sigma_hit * std::erf(.707107 * (z_est) / sigma_hit )) ) +.000000000000000000000001);
    		p_hit = hit_normalizer * hit_dist;
            //std::cout << "hit stuff    " << hit_dist << "            " << hit_normalizer << std::endl;
            if(p_hit > 1) p_hit = .999;
    	}
        
        //std::cout << "z_hit         " << z_hit << "        phit         " << p_hit << std::endl;

    	if ( (0 <= r_las) && (r_las <= z_est) ) {
    		p_short = 1/(1-std::exp(-lambda_short * z_est)) * lambda_short * std::exp(-lambda_short * z_est);
    	}

    	if (r_las >= z_max) {
    		p_max = 1;
    	}

    	if (r_las < z_max) {
    		p_rand = 1 / z_max;
    	}

        

        double q_multiplier = (z_hit * p_hit + z_max * p_max + z_short * p_short + z_rand * p_rand);

        //std::cout << z_hit * p_hit << " " << z_max * p_max << " " << z_short * p_short << " " << z_rand * p_rand << " " << q << " " << q_multiplier << std::endl;

    	q = q * q_multiplier;

	}
    
    std::uniform_real_distribution<double> distribution(0,1);

    
    //std::cout << distribution(rng);
    // q=distribution(rng);
    //std::cout << "q: " << q << std::endl;
    //printf("%f \n", q);
    return q;
}



//raycasting for detecting the expected distance from the pose to the nearest occupied cell in that direction
double SensorModel::raycast_dist(const particle_t& sample, const OccupancyGrid& map, float angle) {
    
    double z_est = z_max;

    
    Point<double> start_position = Point<double>(sample.pose.x, sample.pose.y);

    Point<int> start_cell = global_position_to_grid_cell(start_position, map);
    start_cell.x = start_cell.x;
    
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
    //int th_temp;
    //float self_theta;
    float th_global = angle;

    
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
    switch(quad) {
        
        case 1 :    deltax = 1;//map.widthInCells() - curr_cell.x;
                    deltay = tan(th_global) * deltax;
                    break;

        case 2:     deltay = 1;//map.heightInCells() - curr_cell.y;   
                    deltax = deltay / tan(th_global);
                    break;

        case 3:     deltax = -1;//-1*map.widthInCells() - curr_cell.x;
                    deltay = tan(th_global) * deltax;
                    break;

        case 4:     deltay = -1;//-1*map.heightInCells() - curr_cell.y;   
                    deltax = deltay / tan(th_global);
                    break;
        default:
                    deltax = 1;
                    deltay = 1;
                    break;
    }

    // Boundary checking
    if (deltax == 0) deltaerr = 0;
    else deltaerr = fabs(deltay / deltax);



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
    x = curr_cell.x; // start at self x
    y = curr_cell.y;  // start at self y
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
            //update pos
            curr_pos.x = curr_pos.x + map.metersPerCell();
        }
        else if (quad == 2) {
            y++;
            //update pos
            curr_pos.y = curr_pos.y + map.metersPerCell();
        }
        else if (quad == 3) {
            x--;
            //update pos
            curr_pos.x = curr_pos.x - map.metersPerCell();
        }
        else if (quad == 4) {
            y--;
            //update pos
            curr_pos.y = curr_pos.y - map.metersPerCell();
        }

        //if the cell is occupied break
        if ( map.operator()(x, y) > tip_val) {
            break;
        }

        //update cell
        curr_cell.x = x;
        curr_cell.y = y;

    }
    //end of copied code


    Point<double> end_position  = grid_position_to_global_position( curr_pos, map);
    end_position.x = map.originInGlobalFrame().x + curr_cell.x*map.metersPerCell();     
    end_position.y = map.originInGlobalFrame().y + curr_cell.y*map.metersPerCell();


    //Point<double> start_pose = global_position_to_grid_position(start_position, map);
    //z_est = std::hypot(end_position.x - start_position.x, end_position.y - start_position.y);


    //z_est = std::hypot(  (curr_cell.x-start_cell.x)*map.metersPerCell(), (curr_cell.y-start_cell.y)*map.metersPerCell()  );
    z_est = std::hypot(    (start_position.x - end_position.x), (start_position.y - end_position.y)  );
    //std::cout << "zest    " << z_est << "    ";
    return z_est; 
    //return 1.0; // DEBUG DEBUG DEBUG ***
}


//dumb raycast that just does simple math to test
double SensorModel::simple_raycast_dist(const particle_t& sample, const OccupancyGrid& map, float angle) {

    return 0.0;
}

