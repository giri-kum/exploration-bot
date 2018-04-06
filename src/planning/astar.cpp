#include <planning/astar.hpp>
#include <planning/obstacle_distance_grid.hpp>


robot_path_t search_for_path(pose_xyt_t start, 
                             pose_xyt_t goal, 
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params)
{
    ////////////////// TODO: Implement your A* search here //////////////////////////
    
    // Initialize output
    robot_path_t path;
    path.utime = start.utime;
    path.path.push_back(start);

    // Initialize start and end nodes
    Node nstart(start, 0, -1);
    Node ngoal(goal, 0, -1);



    // Set path size
    path.path_length = path.path.size();
    return path;
}






float * get_motion_model() {
    // dx, dy, cost
    motion = {[1, 0, 1],
              [0, 1, 1],
              [-1, 0, 1],
              [0, -1, 1],
              [-1, -1, sqrt(2)],
              [-1, 1, sqrt(2)],
              [1, -1, sqrt(2)],
              [1, 1, sqrt(2)]};

	return motion

}