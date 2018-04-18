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
    std::map<int, Node>::iterator n_i;

    Node min_node;
    Node current;
    Node new_node;
    std::map<int, Node>::iterator foundNode;

    int width = distances.widthInCells();

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
    //std::vector<Node> openset;
    //std::vector<Node> closedset;
    std::map<int, Node> openset;
    std::map<int, Node> closedset;

    n_ind = calc_id(nstart.x, nstart.y, width);
    openset.insert(std::pair(n_ind, nstart));

    // Plan the path
    while(1) {
        //std::cout << "openset size: " << openset.size() << std::endl;

        // Find the first item on the openset
        min_cost = 100000;
        n_ind = 0;
        // Find item with lowest cost
        for (n_i = openset.begin(); n_i < openset.end(); n_i++) {
            if (openset[n_i].cost < min_cost) {
                min_node = openset[i];
                n_ind = calc_id(openset[n_i].x, openset[n_i].y, width);
            }
        }
        current = openset[n_ind];

        //std::cout << "min_node: " << current.x << ", " << current.y << std::endl;

        if (current.x == ngoal.x && current.y == ngoal.y) {
            std::cout << "found the goal!" << std::endl;
            ngoal.pind = current.pind;
            ngoal.cost = current.cost;
            break;
        }

        // Remove the item from the open set
        foundNode = openset.find(n_ind);
        openset.erase(foundNode);

        //std::cout << n_ind << std::endl;

        // Add it to the closed set
        closedset.insert(n_ind,current);

        // expand search grid based on motion model
        for (i = 0; i < motion_len; i++) {

            // get parent id
            p_ind = calc_id(current.x, current.y, width);

            // Create expanded node
            new_node = Node(current.x + motion[i][0], current.y + motion[i][1],
                        current.cost + motion[i][2], p_ind);

            n_ind = calc_id(new_node.x, new_node.y, width);

            //std::cout << "exploring " << new_node.x << ", " << new_node.y << std::endl;

            // If the node is not on the map or it is an obstacle
            if (!distances.isCellInGrid(new_node.x, new_node.y) || distances(new_node.x, new_node.y) < params.minDistanceToObstacle) continue;

            //stop = false;
            // If the node is in the closed set
            /*for (j = 0; j < (int)closedset.size(); j++) {
                if (closedset[j].x == new_node.x && closedset[j].y == new_node.y) stop = true;
            }
            */
            //if (stop) continue;

            // If node is already in closed set, continue
            foundNode = closedset.find(n_ind);
            if (foundNode != openset.end()) continue;

            // Otherwise, check if it is in the openset and update
            foundNode = openset.find(n_ind);
            // If it is in the openset, update it
            if (foundNode != openset.end()) {
                openset[n_ind] = new_node;
            }
            // otherwise, add it
            else {
                openset.insert(std::pair<int, Node>(n_ind, new_node));
            }

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

int calc_id(int x, int y, int width) {

    return y*width + x;
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