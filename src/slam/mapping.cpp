#include <slam/mapping.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <common/grid_utils.hpp>
#include <numeric>


Mapping::Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds)
: kMaxLaserDistance_(maxLaserDistance)
, kHitOdds_(hitOdds)
, kMissOdds_(missOdds)
{
}


void Mapping::updateMap(const lidar_t& scan, const pose_xyt_t& pose, OccupancyGrid& map)
{
    // Local Variables
    int i;
    int quad;           // quadrants 1-4 (shifted by -PI/4)
    float th_temp;      // angle used to find quadrant
    int not_done;       // 1 if not done, 0 if done

    // Laser Variables
    int r_las;                // distance from self to laser scan termination (m)
    int th_las;               // theta between self and laser scan

    // Self Variables
    float self_theta;    // stores self heading 

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
    self_global.x = pose.x;
    self_global.y = pose.y;
    self_theta = pose.theta;

    // Iterate through scans
    for (i = 0; i < scan.num_ranges; i++) {

        // Extract laser data
        r_las = scan.ranges[i];
        th_las = scan.thetas[i];

        // Get global data **check for bugs**
        th_global = self_theta + th_las;

        // Get final position coordinates
        las_global.x = r * cos(th_global);
        las_global.y = r * sin(th_global);

        // Convert final position to cell
        las_cell = global_position_to_grid_cell(las_global, map);

        // Convert self pose to cell
        self_cell = global_position_to_grid_cell(self_global, map);

        // Find quadrant (1-4)
        th_temp = -PI/4;
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

        while (not_done) {
            
            // Update current cell **check for bugs, possible wrong time to update**
            map(x, y) += kMissOdds_;

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

        // Update last point
        map(las_cell.x, las_cell.y) += kHitOdds_;

    }
    
}
