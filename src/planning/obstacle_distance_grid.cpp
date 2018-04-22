#include <planning/obstacle_distance_grid.hpp>
#include <slam/occupancy_grid.hpp>


ObstacleDistanceGrid::ObstacleDistanceGrid(void)
: width_(0)
, height_(0)
, metersPerCell_(0.05f)
, cellsPerMeter_(20.0f)
{
}


void ObstacleDistanceGrid::setDistances(const OccupancyGrid& map)
{
    resetGrid(map);
    
    ///////////// TODO: Implement an algorithm to mark the distance to the nearest obstacle for every cell in the map.

    // Local Variables
    int i;
    int j;
    int k;
    int m;
    float min_dist;
    float temp_dist;
    float x_tmp;
    float y_tmp;
    int dp; // change in position when checking surrounding cells

    // Loop through grid and compute each distance for each cell
    for(i = 0; i < width_; i++) {
        for (j = 0; j < height_; j++) {

            // If cell is an obstacle, set distance to 0
            if(map(i, j) > 0) distance(i, j) = 0;
            else {
                min_dist = 10000000; // initialize to a large number
                
                // USE CELLS PER METER AS THE UPPER BOUND TO OPTIMIZE -JS
                for (int dp = 1; dp < cellsPerMeter_; dp++) {
                    for (k = i - dp; k <= i + dp; k++) {
                        for (m = j - dp; m <= j + dp; m++) {
                            if (map.isCellInGrid(k,m)) {
                                if (map(k,m) > 0) {
                                    // Find the distance between cells
                                    x_tmp = (i - k)*metersPerCell_;
                                    y_tmp = (j - m)*metersPerCell_;
                                    temp_dist = sqrt(pow(x_tmp, 2) + pow(y_tmp, 2));
                                    if (temp_dist < min_dist) min_dist = temp_dist;
                                }
                            }
                        }
                    }
                    if (min_dist != 10000000) {
                        break; // if found an obstacle
                    }
                }
                //std::cout << "break!" << std::endl;
                /*
                // Check every other cell
                for(k = 0; k < width_; k++) {
                    for (m = 0; m < height_; m++) {
                        // If the cell is an obstacle
                        if (map(k,m) > 0) {
                            // Find the distance between cells
                            x_tmp = (i - k)*metersPerCell_;
                            y_tmp = (j - m)*metersPerCell_;
                            temp_dist = sqrt(pow(x_tmp, 2) + pow(y_tmp, 2));
                            if (temp_dist < min_dist) min_dist = temp_dist;
                        }
                    }
                }
                */
                distance(i, j) = min_dist;
            }  
        }
    }

    // DEBUGGING
    //for(i = 0; i < height_; i++) {
    //    for (j = 0; j < width_; j++) {
    //        std::cout << distance(i,j) << " ";
    //    }
    //    std::cout << std::endl;
    //}
}


bool ObstacleDistanceGrid::isCellInGrid(int x, int y) const
{
    return (x >= 0) && (x < width_) && (y >= 0) && (y < height_);
}


void ObstacleDistanceGrid::resetGrid(const OccupancyGrid& map)
{
    // Ensure the same cell sizes for both grid
    metersPerCell_ = map.metersPerCell();
    cellsPerMeter_ = map.cellsPerMeter();
    globalOrigin_ = map.originInGlobalFrame();
    
    // If the grid is already the correct size, nothing needs to be done
    if((width_ == map.widthInCells()) && (height_ == map.heightInCells()))
    {
        return;
    }
    
    // Otherwise, resize the vector that is storing the data
    width_ = map.widthInCells();
    height_ = map.heightInCells();
    
    cells_.resize(width_ * height_);
}
