#include <slam/mapping.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <common/grid_utils.hpp>
#include <numeric>
#include <iostream>


Mapping::Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds)
: kMaxLaserDistance_(maxLaserDistance)
, kHitOdds_(hitOdds)
, kMissOdds_(missOdds)
{
    // Initialize prevPose?
    prevPose.x = 0;
    prevPose.y = 0;
    prevPose.theta = 0;
    prevPose.utime = 0;

    //std::cout << kHitOdds_ << " " << kMissOdds_ << std::endl;
    //std::cout << hitOdds << " " << missOdds << std::endl;
}


void Mapping::updateMap(const lidar_t& scan, const pose_xyt_t& pose, OccupancyGrid& map)
{
    // Local Variables
    int i;
    int quad;           // quadrants 1-4 (shifted by -PI/4)
    float th_temp;      // angle used to find quadrant
    int not_done;       // 1 if not done, 0 if done

    // Laser Variables
    float r_las;                // distance from self to laser scan termination (m)
    //int th_las;               // theta between self and laser scan
    MovingLaserScan moveLaser(scan, prevPose, pose);
    adjusted_ray_t adjLaser;

    std::cout << pose.x << " " << pose.y << std::endl;

    // Self Variables
    //float self_theta;    // stores self heading 

    // Global Frame Variables
    float th_global;

    // Bresenham's Algorithm variables
    float deltax;
    float deltay;
    float deltaerr;
    float error;
    int x;
    int y;

    // Points
    Point<double> self_global; // self position (m)
    Point<int> self_cell; // self position (cells)
    Point<double> las_global; // point where laser terminates (m)
    Point<int> las_cell; // point where laser terminates (cells)

    // Extract pose data to self
    //self_global.x = adjLaser.origin.x;
    //self_global.y = adjLaser.origin.y;
    //self_theta = pose.theta;
    /*self_global.x = pose.x;
    self_global.y = pose.y;
    self_theta = pose.theta;
    */

/*
    map.operator()(100,130) = -127;
    map.operator()(100,131) = 127;
*/

    
    // Iterate through scans
    for (i = 0; i < scan.num_ranges; i++) {


        //self_theta = adjLaser.theta;

        // Extract laser data
        adjLaser = moveLaser[i];
        r_las = adjLaser.range;
        th_global = adjLaser.theta;

        //r_las = scan.ranges[i];
        //th_las = scan.thetas[i];

        // Extract pose data to self
        self_global.x = adjLaser.origin.x;
        self_global.y = adjLaser.origin.y;

        // Get global data **check for bugs**
        //th_global = self_theta + th_las;

        // Get final position coordinates
        las_global.x = self_global.x + r_las * cos(th_global);
        las_global.y = self_global.y + r_las * sin(th_global);


        // Convert final position to cell
        las_cell = global_position_to_grid_cell(las_global, map);

        // Convert self pose to cell
        self_cell = global_position_to_grid_cell(self_global, map);

        // Find quadrant (1-4)
        //th_temp = -M_PI/4;
        //quad = 1;
        //while (th_global - th_temp > M_PI/2) {
        //    th_temp += M_PI/2;
         //   quad += 1;
        //}
        quad = 1;
        if (th_global < M_PI/4 && th_global > -M_PI/4) quad = 1;
        else if (th_global < 3*M_PI/4 && th_global > M_PI/4) quad = 2;
        else if (th_global > 3*M_PI/4 || th_global < -3*M_PI/4) quad = 3;
        else if (th_global > -3*M_PI/4 && th_global < -M_PI/4) quad = 4;
        //std::cout << "th_global: " << th_global << " quad: " << quad << std::endl;

        // Get properties of line
        deltax = las_cell.x - self_cell.x; //**should not be zero**
        deltay = las_cell.y - self_cell.y;

        // Boundary checking
        if (deltax == 0) deltaerr = 0;
        else deltaerr = fabs(deltay / deltax);

        // Update all cells along line with Bresenham's Line Algorithm
        x = self_cell.x; // start at self x
        y = self_cell.y;  // start at self y
        error = 0; // no error at start

        not_done = 1;
        
        if (quad != 0) {
            while (not_done) {
                
                // Update current cell **check for bugs, possible wrong time to update**
                //map(x, y) += kMissOdds_;
                if (map.operator()(x, y) - kMissOdds_ < -127) map.operator()(x, y) = -127;
                else map.operator()(x,y) -= kMissOdds_;


                //std::cout << th_global << std::endl;

                // Update error
                error += deltaerr;
                while (error >= 0.5) {
                    if (quad == 1 || quad == 3) y += (deltay>0)?1:-1;
                    else x += (deltax>0)?1:-1;
                    error -= 1;
                }

                // Increment and check if done based on quadrant
                if (quad == 1) {
                    x++;
                    not_done = x < las_cell.x;
                }
                else if (quad == 2) {
                    y--;
                    not_done = y > las_cell.y;
                }
                else if (quad == 3) {
                    x--;
                    not_done = x > las_cell.x;
                }
                else if (quad == 4) {
                    y++;
                    not_done = y < las_cell.y;
                }
            }
        }
        
        // Update last point
        //map(las_cell.x, las_cell.y) += kHitOdds_;
        //map.operator()(las_cell.x, las_cell.y) = -127;

        if (map.operator()(las_cell.x, las_cell.y) + kHitOdds_ > 127) map.operator()(las_cell.x, las_cell.y) = 127;
        else map.operator()(las_cell.x, las_cell.y) += kHitOdds_;
        //int8_t cell_out = map.logOdds(las_cell.x, las_cell.y);
        //std::cout << (int)map.logOdds(las_cell.x, las_cell.y) << std::endl;

        
    }

    // Save previous pose
    prevPose = pose;

}
