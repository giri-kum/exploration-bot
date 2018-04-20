#ifndef PLANNING_ASTAR_HPP
#define PLANNING_ASTAR_HPP

#include <lcmtypes/robot_path_t.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <queue>
#include <math.h>
#include <vector>

class ObstacleDistanceGrid;

/**
* SearchParams defines the parameters to use when searching for a path. See associated comments for details
*/
struct SearchParams
{
    double minDistanceToObstacle;   ///< The minimum distance a robot can be from an obstacle before
                                    ///< a collision occurs
                                    
    double maxDistanceWithCost;     ///< The maximum distance from an obstacle that has an associated cost. The planned
                                    ///< path will attempt to stay at least this distance from obstacles unless it must
                                    ///< travel closer to actually find a path
                                    
    double distanceCostExponent;    ///< The exponent to apply to the distance cost, whose function is:
                                    ///<   pow(maxDistanceWithCost - cellDistance, distanceCostExponent)
                                    ///< for cellDistance > minDistanceToObstacle && cellDistance < maxDistanceWithCost
};

/**
* Node for path plan
* -JS
*/
struct Node
{
    int x;    // x position of this node
    int y;    // y position of this node
    float cost;         // cost of this node
    int pind;           // path index
    int inset;      // 1 if in set, 0 if not

    Node() : x(-1), y(-1), cost(0), pind(-1), inset(0) {}
    Node(int x_in, int y_in, float cost_in, int pind_in, int inset_in) : x(x_in), y(y_in), cost(cost_in), pind(pind_in), inset(inset_in) {}

    //bool operator()( const Node* a, const Node* b ) const {
    //    return a->cost < b->cost;
   //}

    bool operator<(const Node &b) const {
        return cost < b.cost;
    }

    bool operator==(const Node &b) const {
        return x == b.x && y == b.y;
    }

};


/**
* search_for_path uses an A* search to find a path from the start to goal poses. The search assumes a circular robot
* 
* \param    start           Starting pose of the robot
* \param    goal            Desired goal pose of the robot
* \param    distances       Distance to the nearest obstacle for each cell in the grid
* \param    params          Parameters specifying the behavior of the A* search
* \return   The path found to the goal, if one exists. If the goal is unreachable, then a path with just the initial
*   pose is returned, per the robot_path_t specification.
*/
robot_path_t search_for_path(pose_xyt_t start, 
                             pose_xyt_t goal, 
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params);

// Calculate heuristic
float calc_h(Node ngoal, int x, int y, const ObstacleDistanceGrid& distances, const SearchParams& params);

// Calculate node id
int calc_id(int x, int y, int width);

// Calculate final path
void calc_final_path(Node ngoal, const Node * closedset, robot_path_t * path, const ObstacleDistanceGrid& distances);

// Print sets for debugging
//void print_sets(const Node * openset, const Node * closedset);


#endif // PLANNING_ASTAR_HPP
