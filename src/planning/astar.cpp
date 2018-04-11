#include <planning/astar.hpp>
#include <planning/obstacle_distance_grid.hpp>


robot_path_t search_for_path(pose_xyt_t start, 
                             pose_xyt_t goal, 
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params)
{    
    // Local Variables
    int i;
    int j;
    int motion_len;
    int stop;
    int n_ind;    
    int p_ind;

    Node min_node;
    Node current;
    Node new_node;

    // Initialize output
    robot_path_t path;
    path.utime = start.utime;
    //path.path.push_back(start);

    // Initialize start and end nodes
    Node nstart(start.x, start.y, 0, -1);
    Node ngoal(goal.x, goal.y, 0, -1);

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
    std::vector<Node> openset;
    std::vector<Node> closedset;

    openset.push_back(nstart);

    // Plan the path
    while(1) {

        //std::cout << "openset size: " << openset.size() << std::endl;

        // Find the first item on the openset
        min_node = openset[0];
        n_ind = 0;
        for (i = 0; i < (int)openset.size(); i++) {
            if (openset[i].cost < min_node.cost) {
                min_node = openset[i];
                n_ind = i;
            }
        }
        current = min_node;

        std::cout << "min_node: " << current.x << ", " << current.y << std::endl;

        if (current.x == ngoal.x && current.y == ngoal.y) {
            std::cout << "found the goal!" << std::endl;
            ngoal.pind = current.pind;
            ngoal.cost = current.cost;
            break;
        }

        // Remove the item from the open set
        openset.erase(openset.begin(), openset.begin() + n_ind + 1);

        std::cout << n_ind << std::endl;

        // Add it to the closed set
        closedset.push_back(current);

        // expand search grid based on motion model
        for (i = 0; i < motion_len; i++) {

            // get parent id
            p_ind = current.y*distances.widthInCells() + current.x;

            new_node = Node(current.x + motion[i][0], current.y + motion[i][1],
                        current.cost + motion[i][2], p_ind);

            //std::cout << "exploring " << new_node.x << ", " << new_node.y << std::endl;

            // If the node is not on the map or it is an obstacle
            if (!distances.isCellInGrid(new_node.x, new_node.y) || distances(new_node.x, new_node.y) < params.minDistanceToObstacle) continue;

            stop = false;
            // If the node is in the closed set
            for (j = 0; j < (int)closedset.size(); j++) {
                if (closedset[j].x == new_node.x && closedset[j].y == new_node.y) stop = true;
            }
            if (stop) continue;

            // Otherwise if it is already in the open set
            for (j = 0; j < (int)openset.size(); j++) {
              // If the node is already in the openset, replace it
              if (openset[j].x == new_node.x && openset[j].y == new_node.y) {
                  if (openset[j].cost > new_node.cost) openset[j] = new_node;
              }
              // otherwise, add it to openset
              else openset.push_back(new_node);
            }
        }

    }
    calc_final_path(ngoal, closedset, &path);

    // Clean up
    //releaseNodes(openset);
    //releaseNodes(closedset);

    // Set path size
    path.path_length = path.path.size();
    return path;
}

float calc_h(Node ngoal, int x, int y) {

    float w = 10.0;  // weight of heuristic
    float d = w * sqrt((ngoal.x - x)*2 + (ngoal.y - y)*2);
    return d;
}

void calc_final_path(Node ngoal, std::vector<Node> closedset, robot_path_t * path) {

    std::cout << "calculating final path!" << std::endl;

    // Local Variables
    pose_xyt_t next;
    Node n;

    // generate final course
    int pind = ngoal.pind;
    while (pind != -1) {
        n = closedset[pind];
        next.x = n.x;
        next.y = n.y;
        path->path.push_back(next);
        pind = n.pind;
    }
}

/*void releaseNodes(Node* nodes)
{
    for (auto it = nodes.begin(); it != nodes.end();) {
        delete *it;
        it = nodes.erase(it);
    }
}*/