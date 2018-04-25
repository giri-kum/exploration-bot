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
    MovingLaserScan moveLaser(scan, prevPose, pose);
    adjusted_ray_t adjLaser;

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
    
    // Iterate through scans
    for (i = 0; i < scan.num_ranges; i++) {

        // Extract laser data
        adjLaser = moveLaser[i];
        r_las = adjLaser.range;
        th_global = adjLaser.theta;

        // Max laser range
        if (r_las > kMaxLaserDistance_) r_las = kMaxLaserDistance_;

        // Extract pose data to self
        self_global.x = adjLaser.origin.x;
        self_global.y = adjLaser.origin.y;

        // Get final position coordinates
        las_global.x = self_global.x + r_las * cos(th_global);
        las_global.y = self_global.y + r_las * sin(th_global);

        // Convert final position to cell
        las_cell = global_position_to_grid_cell(las_global, map);

        // Convert self pose to cell
        self_cell = global_position_to_grid_cell(self_global, map);

        // Check if cells are in grid
        if (!map.isCellInGrid(las_cell.x, las_cell.y) || !map.isCellInGrid(self_cell.x, self_cell.y)) continue;
        
        // Determine quadrant (1-4)
        quad = 0;
        if (th_global < M_PI/4 && th_global >= -M_PI/4) quad = 1;
        else if (th_global < 3*M_PI/4 && th_global >= M_PI/4) quad = 2;
        else if (th_global >= 3*M_PI/4 || th_global < -3*M_PI/4) quad = 3;
        else if (th_global >= -3*M_PI/4 && th_global < -M_PI/4) quad = 4;
        if (quad == 0) {
            std::cout << "error! quad == 0, th_global = " << th_global << ", pose = " << std::endl;
        }
        
        // Get properties of line
        deltax = las_cell.x - self_cell.x;
        deltay = las_cell.y - self_cell.y;

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

        not_done = 1;
        
        // Loop until line is complete
        while (not_done) {
            
            // Update current cell
            if (map.isCellInGrid(x,y)) {
                if (map.operator()(x, y) - kMissOdds_ < -127) map.operator()(x, y) = -127;
                else map.operator()(x,y) -= kMissOdds_;
            }

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
                y++;
                not_done = y < las_cell.y;
            }
            else if (quad == 3) {
                x--;
                not_done = x > las_cell.x;
            }
            else if (quad == 4) {
                y--;
                not_done = y > las_cell.y;
            }
        }

        // Increase the odds of the point where the laser terminates
        if (map.isCellInGrid(las_cell.x, las_cell.y)) {
            if (map.operator()(las_cell.x, las_cell.y) + kHitOdds_ > 127) map.operator()(las_cell.x, las_cell.y) = 127;
            else map.operator()(las_cell.x, las_cell.y) += kHitOdds_;
        }
    }
    // Save previous pose
    prevPose = pose;
}
