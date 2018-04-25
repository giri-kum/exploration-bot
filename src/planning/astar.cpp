#include <planning/astar.hpp>
#include <planning/obstacle_distance_grid.hpp>


robot_path_t search_for_path(pose_xyt_t start, 
                             pose_xyt_t goal, 
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params)
{     
    // Local Variables
    uint i;             // iterator uint
    int n_ind;          // node index
    int p_ind;          // parent index
    int id;

    Node current;
    Node new_node;

    // Get grid parameters
    int width = distances.widthInCells();
    int height = distances.heightInCells();
    int tot_nodes = calc_id(width, height, width);

    // Get start cells
    Point<int> startCell = global_to_cell(start.x, start.y, distances);
    Point<int> goalCell = global_to_cell(goal.x, goal.y, distances);

    // Heuristic variables
    float check_h;
    float current_h;

    // Initialize output
    robot_path_t path;
    path.utime = start.utime;
    path.path.push_back(start);     // add first pose

    // Initialize start and end nodes
    Node nstart(startCell.x, startCell.y, 0, -1, 1);
    Node ngoal(goalCell.x, goalCell.y, 0, -1, 0);

    // Motion model: x, y, cost
    uint motion_len = 8;
    float motion[8][3] = {{1, 0, 1},
          {0, 1, 1},
          {-1, 0, 1},
          {0, -1, 1},
          {-1, -1, sqrt(2)},
          {-1, 1, sqrt(2)},
          {1, -1, sqrt(2)},
          {1, 1, sqrt(2)}};

    // Initialize open set and closed set
    Node openset[tot_nodes];
    Node closedset[tot_nodes];
    int open_size = 0;
    int closed_size = 0;

    // add nstart to the openset
    id = calc_id(startCell.x, startCell.y, width);
    openset[id] = nstart;
    open_size++;

    // Plan the path
    while(1) {

        // set first node to first inset node in openset
        if (open_size != 0) {
            for (i = 0; i < tot_nodes; i++) {
                if (openset[i].inset) {
                    current = openset[i];
                    break;
                }
            }
        }
        else {
            return path;
        }
        n_ind = i; // reset n_ind        

        // Find item with lowest cost
        for (i += 1; i < tot_nodes; i++) {
            // if node is in the open set
            if (openset[i].inset) {
                // Calculate heuristics
                check_h = calc_h(ngoal, openset[i].x, openset[i].y, distances, params);
                current_h = calc_h(ngoal, current.x, current.y, distances, params);

                if (openset[i].cost + check_h < current.cost + current_h) { // if the node has a lower cost than the saved node
                    n_ind = i;
                    current = openset[i];
                }
            }
        }

        // Check if we're at the goal
        if (current == ngoal) {
            ngoal.pind = current.pind;
            break;
        }

        // Add it to the closed set
        closedset[n_ind] = current;
        closed_size++;

        // Remove the item from the open set
        openset[n_ind].inset = 0;
        open_size--;

        // get parent id
        p_ind = n_ind;

        // expand search grid based on motion model
        for (i = 0; i < motion_len; i++) {

            // Create expanded node
            new_node = Node(current.x + motion[i][0], current.y + motion[i][1],
                        current.cost + motion[i][2], p_ind, 1);

            // add heuristic
            n_ind = calc_id(new_node.x, new_node.y, width);

            // If the grid is not empty and (the node is not on the map or it is an obstacle)
            if (!distances.isCellInGrid(new_node.x, new_node.y)) continue;
            
            // Check if cell is safe
            if (distances(new_node.x, new_node.y) < params.minDistanceToObstacle*1.5) continue;

            // If node is already in closed set, continue
            if (closedset[n_ind].inset == 1) continue;

            // Otherwise, check if it is in the openset and update
            if (!openset[n_ind].inset) open_size++;
            openset[n_ind] = new_node;
        }
    }
    calc_final_path(ngoal, closedset, &path, distances);

    // Set path size
    path.path_length = path.path.size();
    return path;
}

float calc_h(Node ngoal, int x, int y, const ObstacleDistanceGrid& distances, const SearchParams& params) {

    // Distance from goal cost
    float w1 = 10;  // weight of heuristic
    float d = w1 * sqrt(pow((ngoal.x - x), 2) + pow((ngoal.y - y), 2));
    
    // Add obstacle distance cost
    float w2 = 1.5;
    float od = 0;
    if (distances(x, y) <= params.maxDistanceWithCost) {
        od = w2 * pow(distances.cellsPerMeter() * (params.maxDistanceWithCost - distances(x, y)), params.distanceCostExponent);
    }

    return d + od;
}

int calc_id(int x, int y, int width) {

    return y*width + x;
}

void calc_final_path(Node ngoal, const Node * closedset, robot_path_t * path, const ObstacleDistanceGrid& distances) {

    // Local Variables
    pose_xyt_t next; // position in world coordinates
    pose_xyt_t turn; // position in world coordinates
    Node n;             // node
    int path_size = 0;
    robot_path_t temp_path;
    float theta;
    int j;          //iterator

    Point<float> goal_point = cell_to_global(ngoal.x, ngoal.y, distances);
    next.x = goal_point.x;
    next.y = goal_point.y;

    temp_path.path.push_back(next);
    path_size++;

    // generate final course
    int pind = ngoal.pind;
    while (pind != -1) {

        n = closedset[pind];

        // add next coordinate to temp_path
        Point<float> next_point = cell_to_global(n.x, n.y, distances);
        next.x = next_point.x;
        next.y = next_point.y;

        temp_path.path.push_back(next);
        pind = n.pind;

        path_size++;

    }

    // add second pose to path
    if (path_size > 1) {

        next = temp_path.path[path_size - 2];
        Point<int> next_cell = global_to_cell(next.x, next.y, distances);
        Point<int> prev_cell = global_to_cell(path->path[0].x, path->path[0].y, distances);
        theta = atan2(next_cell.y - prev_cell.y, next_cell.x - prev_cell.x);

        next.theta = theta;
        path->path.push_back(next);
    }
    else return;

    // trim and reverse path (path already contains the first 2 poses)
    // add intermediate points
    j = 2; // place in path
    for (int i = 2; i < path_size; i++) {

        // get pose
        next = temp_path.path[path_size - i - 1];
        
        Point<int> next_cell = global_to_cell(next.x, next.y, distances);
        Point<int> prev_cell = global_to_cell(path->path[j-1].x, path->path[j-1].y, distances);
        theta = atan2(next_cell.y - prev_cell.y, next_cell.x - prev_cell.x);
        next.theta = theta;
        
        // Check if point is the same angle
        if (theta == path->path[j - 1].theta) path->path[j - 1] = next;
        else {

            // add point
            path->path.push_back(next);
            j++; // iterate
        }
    }

}

Point<int> global_to_cell(float x, float y, const ObstacleDistanceGrid& distances) {
    
    Point<int> output;
    output.x  = static_cast<int>((x - distances.originInGlobalFrame().x) * distances.cellsPerMeter());
    output.y = static_cast<int>((y - distances.originInGlobalFrame().y) * distances.cellsPerMeter());

    return output;
}

Point<float> cell_to_global (int x, int y, const ObstacleDistanceGrid& distances) {
    
    Point<float> output;
    output.x = distances.originInGlobalFrame().x + x*distances.metersPerCell();
    output.y = distances.originInGlobalFrame().y + y*distances.metersPerCell();

    return output;
}