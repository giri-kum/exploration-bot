#include <planning/astar.hpp>
#include <planning/obstacle_distance_grid.hpp>


robot_path_t search_for_path(pose_xyt_t start, 
                             pose_xyt_t goal, 
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params)
{     
    // Local Variables
    uint i;             // iterator uint
    uint j;             // iterator uint
    int stop;           // signals loops to exit
    int n_ind;          // node index
    int p_ind;          // parent index
    int id;

    Node current;
    Node new_node;

    // Get grid parameters
    int width = distances.widthInCells();
    int height = distances.heightInCells();
    int tot_nodes = calc_id(width, height, width);

    // PRINT DISTANCES FOR DEBUGGING
    //for (i = 0; i < height; i++) {
    //    for (j = 0; j < width; j++) {
    //        std::cout << distances(i, j) << " ";
    //    }
    //    std::cout << std::endl;
    //}

    // Get start cells
    int startCellx = (start.x - distances.originInGlobalFrame().x) * distances.cellsPerMeter();
    int startCelly = (start.y - distances.originInGlobalFrame().y) * distances.cellsPerMeter();
    int goalCellx = (goal.x - distances.originInGlobalFrame().x) * distances.cellsPerMeter();
    int goalCelly = (goal.y - distances.originInGlobalFrame().y) * distances.cellsPerMeter();

    // Heuristic variables
    float check_h;
    float current_h;

    // Initialize output
    robot_path_t path;
    path.utime = start.utime;
    path.path.push_back(start);     // add first pose

    // Initialize start and end nodes
    Node nstart(startCellx, startCelly, 0, -1, 1);
    Node ngoal(goalCellx, goalCelly, 0, -1, 0);

    //std::cout << "start: (" << nstart.x << ", " << nstart.y << ")\n";
    //std::cout << "goal: (" << ngoal.x << ", " << ngoal.y << ")\n";

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
    //std::vector<Node> openset;
    //std::vector<Node> closedset;
    Node openset[tot_nodes];
    Node closedset[tot_nodes];
    int open_size = 0;
    int closed_size = 0;

    //n_ind = calc_id(nstart.x, nstart.y, width);
    //openset.push_back(nstart);  // add nstart to the openset

    // add nstart to the openset
    id = calc_id(startCellx, startCelly, width);
    openset[id] = nstart;
    open_size++;

    // Plan the path
    while(1) {
        //std::cout << "openset size: " << openset.size() << std::endl;

        // Find the first item on the openset //

        //std::cout << "starting here!\n";

        // set first node to first inset node in openset
        if (open_size != 0) {
            for (i = 0; i < tot_nodes; i++) {
                if (openset[i].inset) {
                    current = openset[i];
                    //std::cout << "min_node: " << current.x << ", " << current.y << " cost: " << current.cost << std::endl;
                    break;
                }
            }
        }
        else {
            //std::cout << "openset is empty!\n";
            break;
        }
        n_ind = i; // reset n_ind        

        // Find item with lowest cost
        for (; i < tot_nodes; i++) {
            // if node is in the open set
            if (openset[i].inset) {
                // Calculate heuristics
                check_h = calc_h(ngoal, openset[i].x, openset[i].y, distances, params);
                current_h = calc_h(ngoal, current.x, current.y, distances, params);

                if (openset[i].cost + check_h < current.cost + current_h) { // if the node has a lower cost than the saved node
                    n_ind = i;
                    current = openset[i];
                    //std::cout << i << std::endl;
                }
            }
        }
        //current = openset[n_ind];

        //std::cout << "min_node: " << current.x << ", " << current.y << " cost: " << current.cost << std::endl;

        // Check if we're at the goal
        if (current == ngoal) {
            //std::cout << "found the goal!" << std::endl;
            ngoal.pind = current.pind;
            //ngoal.cost = current.cost;
            break;
        }

        // Remove the item from the open set
        //openset.erase(openset.begin() + n_ind);
        //openset[n_ind].inset = 0;

        //std::cout << n_ind << std::endl;

        // Add it to the closed set
        //closedset.push_back(current);
        closedset[n_ind] = current;
        closed_size++;
        // SAVE CLOSEDSET INDEX

        // Remove the item from the open set
        openset[n_ind].inset = 0;
        open_size--;

        //print_sets(openset, closedset); // DEBUG

        // get parent id
        //p_ind = calc_id(current.x, current.y, width);
        //p_ind = (int)closedset.size() - 1;
        //std::cout << p_ind << std::endl;
        p_ind = n_ind;

        // expand search grid based on motion model
        for (i = 0; i < motion_len; i++) {

            //std::cout << "motion model " << i << std::endl;

            // Create expanded node
            new_node = Node(current.x + motion[i][0], current.y + motion[i][1],
                        current.cost + motion[i][2], p_ind, 1);

            //std::cout << "new node cost: " << new_node.cost;

            // add heuristic
            //new_node.cost += calc_h(ngoal, new_node.x, new_node.y);

            //std::cout << " , w/h = " << new_node.cost << std::endl;

            n_ind = calc_id(new_node.x, new_node.y, width);

            //std::cout << "exploring " << new_node.x << ", " << new_node.y << std::endl;

            // If the grid is not empty and (the node is not on the map or it is an obstacle)
            if (!distances.isCellInGrid(new_node.x, new_node.y)) continue;
            //if (distances(new_node.x, new_node.y) >= 0 
                //&& distances(new_node.x, new_node.y) <= params.minDistanceToObstacle) continue;
            if (distances(new_node.x, new_node.y) < params.minDistanceToObstacle) continue;

            // Add distance cost
            //new_node.cost += pow(params.maxDistanceWithCost - distances(new_node.x, new_node.y), params.distanceCostExponent);

            //stop = false;
            // If the node is in the closed set
            /*for (j = 0; j < (int)closedset.size(); j++) {
                if (closedset[j].x == new_node.x && closedset[j].y == new_node.y) stop = true;
            }
            */
            //if (stop) continue;

            // If node is already in closed set, continue
            if (closedset[n_ind].inset == 1) continue;

            //stop = false;
            //for (j = 0; j < closedset.size(); j++) {
            //    if (closedset[j] == new_node) {
            //        stop = true;
            //        break;
            //    }
            //}
            //if (stop) continue;

            // Otherwise, check if it is in the openset and update
            if (!openset[n_ind].inset) open_size++;
            openset[n_ind] = new_node;

            //stop = false;
            //for (j = 0; j < openset.size(); j++) {
            //    if (openset[j] == new_node) {
            //        stop = true;
            //        break;
            //    }
            //}
            //if (stop) openset[j] = new_node;
            //else openset.push_back(new_node);


            /*
            // Otherwise if it is already in the open set
            for (j = 0; j < (int)openset.size(); j++) {
              // If the node is already in the openset, replace it
              if (openset[j].x == new_node.x && openset[j].y == new_node.y) {
                  if (openset[j].cost > new_node.cost) openset[j] = new_node;
              }
              // otherwise, add it to openset
              else openset.push_back(new_node);
            }
            */
        }

    }
    calc_final_path(ngoal, closedset, &path, distances);

    // Check final theta
    if (path.path[j].theta != goal.theta) {
        pose_xyt_t final_pose;
        final_pose.x = path.path[j].x;
        final_pose.y = path.path[j].y;
        final_pose.theta = path.path[j].theta;
        path.path.push_back(final_pose);
    }

    // Clean up
    //releaseNodes(openset);
    //releaseNodes(closedset);

    // Set path size
    path.path_length = path.path.size();
    return path;
}

float calc_h(Node ngoal, int x, int y, const ObstacleDistanceGrid& distances, const SearchParams& params) {

    //std::cout << "x: " << (ngoal.x - x)*2 << " y: " << (ngoal.y - y)*2 << std::endl;

    // Distance from goal cost
    float w1 = 10;  // weight of heuristic
    float d = w1 * sqrt(pow((ngoal.x - x), 2) + pow((ngoal.y - y), 2));
    
    // Add obstacle distance cost
    float w2 = 0.5;
    float od = w2 * pow(params.maxDistanceWithCost - distances(x, y), params.distanceCostExponent);
    
    return d + od;
}

int calc_id(int x, int y, int width) {

    return y*width + x;
}

void calc_final_path(Node ngoal, const Node * closedset, robot_path_t * path, const ObstacleDistanceGrid& distances) {

    //std::cout << "calculating final path!" << std::endl;

    // Local Variables
    pose_xyt_t next; // position in world coordinates
    //pose_xyt_t turn; // position in world coordinates
    Node n;             // node
    int path_size = 0;
    robot_path_t temp_path;
    float theta;
    int j;          //iterator

    /*std::cout << "\nClosedset: ";
    for (uint i = 0; i < closedset.size(); i++) {
        std::cout << closedset[i].pind << std::endl;
    }
    std::cout << std::endl << std::endl;*/

    // debug
    //std::cout << "final path (backwards): ";

    //float ngoalx = distances.originInGlobalFrame().x + ngoal.x*distances.metersPerCell();
    //float ngoaly = distances.originInGlobalFrame().y + ngoal.y*distances.metersPerCell();
    //std::cout << "(" << ngoalx << ", " << ngoaly << ") ";

    // add goal point
    next.x = distances.originInGlobalFrame().x + ngoal.x*distances.metersPerCell();
    next.y = distances.originInGlobalFrame().y + ngoal.y*distances.metersPerCell();
    temp_path.path.push_back(next);
    path_size++;

    // generate final course
    int pind = ngoal.pind;
    //std::cout << pind << std::endl;
    while (pind != -1) {

        //for (int i = 0; i < closedset.size(); i++) {
        //    if (calc_id(closedset[i].x, closedset[i].y, distances.widthInCells()) == pind) //{}
        //        n = closedset[i];
        //       //std::cout << "found next point" << std::endl;
        //        break;
       //}
        n = closedset[pind];

        // add next coordinate to temp_path
        next.x = distances.originInGlobalFrame().x + n.x*distances.metersPerCell();
        next.y = distances.originInGlobalFrame().y + n.y*distances.metersPerCell();
        temp_path.path.push_back(next);
        pind = n.pind;

        path_size++;

        //std::cout << "(" << next.x << ", " << next.y << ") , pind = " << pind << std::endl;;
    }
    //std::cout << "\n";

    // add second pose to path
    if (path_size > 1) {
        next = temp_path.path[path_size - 2];
        theta = atan2(next.y - path->path[0].y, next.x - path->path[0].x);
        next.theta = theta;
        path->path.push_back(next);
    }
    else return;

    //std::cout << path_size << std::endl;
    //std::cout << "(" << next.x << ", " << next.y << ") , pind = " << pind << std::endl;;

    // trim and reverse path (path already contains the first 2 poses)
    // add intermediate points
    j = 2; // place in path
    for (int i = 2; i < path_size; i++) {

        // get pose
        next = temp_path.path[path_size - i - 1];
        
        // Check if point is on the same line
        theta = atan2(next.y - path->path[j - 1].y, next.x - path->path[j - 1].x);
        next.theta = theta;
        
        // Check if point is the same angle
        if (theta == path->path[j - 1].theta) path->path[j - 1] = next;
        else {

            // add point
            //path->path.push_back(temp_path.path[path_size-i-1]);
            path->path.push_back(next);
            j++; // iterate

            // add turn
            //turn = path->path[j - 1];
            //turn.theta = theta;
            //path->path.push_back(turn);
            //j++;
        }
    }

}

/*
void print_sets(const Node * openset, const Node * closedset) {

    std::cout << "Openset size: " << openset.size() << std::endl;
    std::cout << "Closedset size: " << closedset.size() << std::endl;

    std::cout << "Openset: ";
    for (uint i = 0; i < openset.size(); i++) {
        std::cout << "(" << openset[i].x << "," << openset[i].y << "," << openset[i].cost << "), ";
    }

    std::cout << "\nClosedset: ";
    for (uint i = 0; i < closedset.size(); i++) {
        std::cout << "(" << closedset[i].x << "," << closedset[i].y << "," << closedset[i].cost << "), ";
    }
    std::cout << std::endl << std::endl;
}
*/

/*void releaseNodes(Node* nodes)
{
    for (auto it = nodes.begin(); it != nodes.end();) {
        delete *it;
        it = nodes.erase(it);
    }
}*/