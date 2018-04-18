#include <planning/astar.hpp>
#include <planning/obstacle_distance_grid.hpp>


robot_path_t search_for_path(pose_xyt_t start, 
                             pose_xyt_t goal, 
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params)
{     
    // Local Variables
    uint i;
    uint j;
    uint motion_len;
    int stop; 
    int n_ind;  
    int p_ind;

    Node current;
    Node new_node;

    int width = distances.widthInCells();

    int startCellx = (start.x - distances.originInGlobalFrame().x) * distances.cellsPerMeter();
    int startCelly = (start.y - distances.originInGlobalFrame().y) * distances.cellsPerMeter();
    int goalCellx = (goal.x - distances.originInGlobalFrame().x) * distances.cellsPerMeter();
    int goalCelly = (goal.y - distances.originInGlobalFrame().y) * distances.cellsPerMeter();

    // Initialize output
    robot_path_t path;
    path.utime = start.utime;
    //path.path.push_back(start);

    // Initialize start and end nodes
    Node nstart(startCellx, startCelly, 0, -1);
    Node ngoal(goalCellx, goalCelly, 0, -1);

    std::cout << "start: (" << nstart.x << ", " << nstart.y << ")\n";
    std::cout << "goal: (" << ngoal.x << ", " << ngoal.y << ")\n";

    // Motion model: x, y, cost
    motion_len = 8;
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
    std::vector<Node> openset;
    std::vector<Node> closedset;

    n_ind = calc_id(nstart.x, nstart.y, width);
    openset.push_back(nstart);

    // Plan the path
    while(1) {
        //std::cout << "openset size: " << openset.size() << std::endl;

        // Find the first item on the openset //

        //std::cout << "starting here!\n";

        // set first node to first node in openset
        if (openset.size() != 0) current = openset[0];
        else std::cout << "openset is empty!\n";
        n_ind = 0; // reset n_ind        

        // Find item with lowest cost
        for (i = 1; i < openset.size(); i++) {
            if (openset[i].cost < current.cost) { // if the node has a lower cost than the saved node
                n_ind = i;
                //std::cout << i << std::endl;
            }
        }
        current = openset[n_ind];

        //std::cout << "min_node: " << current.x << ", " << current.y << " cost: " << current.cost << std::endl;

        if (current == ngoal) {
            //std::cout << "found the goal!" << std::endl;
            ngoal.pind = current.pind;
            //ngoal.cost = current.cost;
            break;
        }

        // Remove the item from the open set
        openset.erase(openset.begin() + n_ind);

        //std::cout << n_ind << std::endl;

        // Add it to the closed set
        closedset.push_back(current);

        //print_sets(openset, closedset); // DEBUG

        // print size of grid (distances)
        //std::cout <<

        // get parent id
        p_ind = calc_id(current.x, current.y, width);

        // expand search grid based on motion model
        for (i = 0; i < motion_len; i++) {

            //std::cout << "motion model " << i << std::endl;

            // Create expanded node
            new_node = Node(current.x + motion[i][0], current.y + motion[i][1],
                        current.cost + motion[i][2], p_ind);

            //std::cout << "new node cost: " << new_node.cost;

            // add heuristic
            new_node.cost += calc_h(ngoal, new_node.x, new_node.y);

            //std::cout << " , w/h = " << new_node.cost << std::endl;

            n_ind = calc_id(new_node.x, new_node.y, width);

            //std::cout << "exploring " << new_node.x << ", " << new_node.y << std::endl;

            // If the grid is not empty and (the node is not on the map or it is an obstacle)
            if (distances(new_node.x, new_node.y) > 0 
                && (!distances.isCellInGrid(new_node.x, new_node.y) 
                || distances(new_node.x, new_node.y) < params.minDistanceToObstacle)) continue;

            //stop = false;
            // If the node is in the closed set
            /*for (j = 0; j < (int)closedset.size(); j++) {
                if (closedset[j].x == new_node.x && closedset[j].y == new_node.y) stop = true;
            }
            */
            //if (stop) continue;

            // If node is already in closed set, continue
            stop = false;
            for (j = 0; j < closedset.size(); j++) {
                if (closedset[j] == new_node) {
                    stop = true;
                    break;
                }
            }
            if (stop) continue;

            // Otherwise, check if it is in the openset and update
            stop = false;
            for (j = 0; j < openset.size(); j++) {
                if (openset[j] == new_node) {
                    stop = true;
                    break;
                }
            }
            if (stop) openset[j] = new_node;
            else openset.push_back(new_node);


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

    // Clean up
    //releaseNodes(openset);
    //releaseNodes(closedset);

    // Set path size
    path.path_length = path.path.size();
    return path;
}

float calc_h(Node ngoal, int x, int y) {

    //std::cout << "x: " << (ngoal.x - x)*2 << " y: " << (ngoal.y - y)*2 << std::endl;

    float w = 1.0;  // weight of heuristic
    float d = w * sqrt(pow((ngoal.x - x), 2) + pow((ngoal.y - y), 2));
    return d;
}

int calc_id(int x, int y, int width) {

    return y*width + x;
}

void calc_final_path(Node ngoal, const std::vector<Node> &closedset, robot_path_t * path, const ObstacleDistanceGrid& distances) {

    //std::cout << "calculating final path!" << std::endl;

    // Local Variables
    pose_xyt_t next; // position in world coordinates
    Node n;             // node
    int path_size = 0;
    robot_path_t temp_path;

    /*std::cout << "\nClosedset: ";
    for (uint i = 0; i < closedset.size(); i++) {
        std::cout << closedset[i].pind << std::endl;
    }
    std::cout << std::endl << std::endl;*/

    // debug
    //std::cout << "final path (backwards): ";

    float ngoalx = distances.originInGlobalFrame().x + ngoal.x*distances.metersPerCell();
    float ngoaly = distances.originInGlobalFrame().y + ngoal.y*distances.metersPerCell();
    //std::cout << "(" << ngoalx << ", " << ngoaly << ") ";

    next.x = ngoalx;
    next.y = ngoaly;
    temp_path.path.push_back(next);
    path_size++;

    // generate final course
    int pind = ngoal.pind;
    //std::cout << pind << std::endl;
    while (pind != -1) {

        for (int i = 0; i < closedset.size(); i++) {
            if (calc_id(closedset[i].x, closedset[i].y, distances.widthInCells()) == pind) {}
                n = closedset[i];
                //std::cout << "found next point" << std::endl;
                break;
       }
        
        next.x = distances.originInGlobalFrame().x + n.x*distances.metersPerCell();
        next.y = distances.originInGlobalFrame().y + n.y*distances.metersPerCell();
        temp_path.path.push_back(next);
        pind = n.pind;

        path_size++;

        std::cout << "(" << next.x << ", " << next.y << ") , pind = " << pind << std::endl;;
    }
    //std::cout << "\n";

    // reverse path
    for (int i = 0; i < path_size; i++) {
        path->path.push_back(temp_path.path[path_size-i-1]);
    }
}

void print_sets(const std::vector<Node> &openset, const std::vector<Node> &closedset) {

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


/*void releaseNodes(Node* nodes)
{
    for (auto it = nodes.begin(); it != nodes.end();) {
        delete *it;
        it = nodes.erase(it);
    }
}*/