// This file is part of the 2019 skeleton code(https://github.com/mlab-upenn/f110-fall2019-skeletons) 
// While the skeleton code provided an initial structure, the majority of the code 
// was written by Steffen Fleischmann, Jakob Greten, Kilian Poeschel and Joshua Bahn.

// This class creates an rrt tree and calculates a trajectory to a given goal.
#include "rrt/rrt.h"
#include <random>
#include <iostream>
using namespace std;

// Destructor of the RRT class
RRT::~RRT()
{
    ROS_INFO("RRT shutting down");
}

// Constructor of the RRT class. Initiates subscribers and publishers.
RRT::RRT(ros::NodeHandle &nh) : nh_(nh), gen((std::random_device())())
{
    //get RRT Parameters
    nh_.getParam("rrt/pose_topic", pose_topic);
    nh_.getParam("rrt/scan_topic", scan_topic);
    nh_.getParam("rrt/path_topic", path_topic);
    nh_.getParam("rrt/map_buffed_topic", map_topic);
    nh_.getParam("rrt/clicked_point_topic", clicked_point_topic);
    nh_.getParam("rrt/nav_goal_topic", nav_goal_topic);
    nh_.getParam("rrt/marker_topic", marker_topic);
    nh_.getParam("rrt/tree_topic", tree_topic);

    nh_.getParam("rrt/rrt_steps", rrt_steps);
    nh_.getParam("rrt/max_rrt_iterations", max_rrt_iterations);
    nh_.getParam("rrt/collision_accuracy", collision_accuracy);
    nh_.getParam("rrt/step_length", step_length);
    nh_.getParam("rrt/goal_threshold", goal_threshold);
    nh_.getParam("rrt/dRRT", dRRT);
    nh_.getParam("rrt/rrt_bias", rrt_bias);

    //differentiate between simualted and real launch
    bool real = false;
    std::string real_pose_topic = "";
    nh_.getParam("/real", real);
    nh_.getParam("/real_pose_topic", real_pose_topic);
    if(real){
        pose_topic = real_pose_topic;
        ROS_INFO_STREAM("Real RRT launch");
    }


    ROS_INFO_STREAM("rrt_steps: " << rrt_steps << " pose_topic: " << pose_topic);

    // ROS publishers
    vis_pub = nh_.advertise<visualization_msgs::Marker>(marker_topic, 0);
    tree_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(tree_topic, 0);
    path_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(path_topic, 10);

    // ROS subscribers
    pf_sub_ = nh_.subscribe(pose_topic, 10, &RRT::pf_callback, this);
    scan_sub_ = nh_.subscribe(scan_topic, 10, &RRT::scan_callback, this);
    map_sub = nh_.subscribe(map_topic, 1, &RRT::map_callback, this);
    click_sub = nh_.subscribe(clicked_point_topic, 1, &RRT::clicked_point_callback, this);
    nav_goal_sub = nh_.subscribe(nav_goal_topic, 1, &RRT::nav_goal_callback, this);

    //random number generator
    gen = std::mt19937(123);
    x_dist = std::uniform_real_distribution<double>(-35.0, 35.0);
    y_dist = std::uniform_real_distribution<double>(-35.0, 35.0);

    rrt_tree_built = false;

    //starting goal
    q_goal.push_back(22.0);
    q_goal.push_back(-1.8);

    ROS_INFO("Created new RRT Object.");
}

// The loop where the rrt tree is created
void RRT::rrt_loop()
{
    tree.clear();
    Node start;
    start.x = pose_x;
    start.y = pose_y;
    start.cost = 0;
    start.parent = -1;
    start.old_parent = -1;
    start.is_root = true;
    tree.push_back(start);

    bool use_rrt_star=true; //true --> RRT* | false --> RRT

    int counter = 0;
    int iteration =0;
    //end path finding after rrt_steps is reached
    while (counter < rrt_steps && iteration < max_rrt_iterations)
    {
        //generate random sample point
        std::vector<double> sampled_point = sample();

        //get nearest node
        int near = nearest(tree, sampled_point);
        
        //steer towards that node
        Node x_new = steer(tree[near], sampled_point);
        
        //check if node is directly behind the car
        double dist_to_pose = sqrt(pow(pose_x-x_new.x,2)+pow(pose_y-x_new.y,2));
        //angle between orientation of car and x_new
        double alpha = atan2((x_new.y - pose_y), (x_new.x - pose_x)) - pose_theta;
        bool behind_car = dist_to_pose<0.4 && fabs(alpha)>0.6;       

        //check for collisions
        if (!check_collision(tree[near], x_new) && !behind_car)
        {  
            if(use_rrt_star)                           
            {
                rrt_star(tree,x_new,near);
            }
            else
            {
                tree.push_back(x_new);
            }  

            //check if goal is reached
            if (is_goal(x_new))
            {
                path.clear();          
                path = find_path(tree, x_new);
            }   
            counter++;         
        }
        iteration++;              
        
        // Output of the RRT creation process
        if (counter*100/rrt_steps != (counter+1)*100/rrt_steps){
             ROS_INFO_STREAM("RRT Process:"<< counter*100/rrt_steps<<"%\n"
                             <<"Iteration: "<<iteration);
        }
    }

    //calculate path if found
    if (!path.empty())
    {
        Node goalNode = path.front();
        path.clear();          
        path = find_path(tree, goalNode);
        ROS_INFO_STREAM("RRT has finished.\nA path has been found!\nPress \"p\" for Pure Pursuit.");
    }else
    {
        ROS_INFO_STREAM("RRT has finished.\nNo path has been found!");
    }

    ROS_INFO_STREAM("RRT finished on iteration: "<<iteration);

    
    rrt_tree_built = true;
}

// The scan callback, currently not used.
    // Args:
    //    scan_msg (*LaserScan): pointer to the incoming scan message
    
void RRT::scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    /*std_msgs::Float64MultiArray path_msg;

    for (int i = 0; i < path.size(); i++)
    {

        path_msg.data.push_back(path[i].x);
        path_msg.data.push_back(path[i].y);
    }
    path_pub_.publish(path_msg);
    pub_tree(tree);*/
}



// The pose callback when subscribed to pose topic. Publishes path and tree.
    // Args:
    //    pose_msg (*PoseStamped): pointer to the incoming pose message
void RRT::pf_callback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg)
{
    //update pose
    pose_x = pose_msg->pose.position.x;
    pose_y = pose_msg->pose.position.y;
    
    //current angle of car
    pose_theta = tf::getYaw(pose_msg->pose.orientation);
    

    if(path.size()>0){
        //prepare and publish path message
        std_msgs::Float64MultiArray path_msg;

        for (int i = 0; i < path.size(); i++)
        {

            path_msg.data.push_back(path[i].x);
           path_msg.data.push_back(path[i].y);
        }
        path_pub_.publish(path_msg);

        
    }
    //publish rrt tree
    pub_tree(tree);

}

//callback for rviz location. Used for debugging
// Args:
    //    pose_msg (*PoseStamped): pointer to the incoming pose message
void RRT::clicked_point_callback(const geometry_msgs::PointStamped &pose_msg)
{

    //double distance_to_nearest = distance_transform(pose_msg.point.x, pose_msg.point.y);
    //ROS_INFO_STREAM("Distance: " << distance_to_nearest);
    
}

//callback for the new nav goal
// Args:
    //    pose_msg (*PoseStamped): pointer to the incoming pose message
void RRT::nav_goal_callback(const geometry_msgs::PoseStamped &pose_msg)
{
    double x = pose_msg.pose.position.x;
    double y = pose_msg.pose.position.y;

    q_goal.clear();
    q_goal.push_back(x);
    q_goal.push_back(y);
    ROS_INFO_STREAM("New nav goal set to X: " << x << " Y: " << y);
    tree.clear();
    path.clear();

    //if the nav goal is inside of an obstacle, do not look for path
    if(distance_transform(x, y) == 0){
        ROS_INFO_STREAM("New nav goal is located in obstacle. No path finding possible");
    }else{
        rrt_loop();
    }
}

// This method returns a sampled point from the free space
    // Returns:
    //     sampled_point (std::vector<double>): the sampled point in free space
std::vector<double> RRT::sample()
{
    
    std::vector<double> sampled_point;   
    sampled_point.push_back(x_dist(gen));
    sampled_point.push_back(y_dist(gen));
    
    // Create bias by setting some of the samples to the goal position with the possibility 
    // set in the parameter rrt_bias
    if (rand()<=RAND_MAX*rrt_bias && path.empty()){
        sampled_point[0] = q_goal[0];
        sampled_point[1] = q_goal[1];
    }

    return sampled_point;
}

// This method returns the nearest node on the tree to the sampled point
    // Args:
    //     tree (std::vector<Node>): the current RRT tree
    //     sampled_point (std::vector<double>): the sampled point in free space
    // Returns:
    //     nearest_node (int): Index of nearest node on the tree
int RRT::nearest(std::vector<Node> &tree, std::vector<double> &sampled_point)
{
    
    int nearest_node = 0;
    double nearest_distance = 9999999;

    //iterate through tree and find the nearest node
    for (int i = 0; i < tree.size(); i++)
    {

        double distance = distanceNodePoint(tree[i], sampled_point);
        if (distance < nearest_distance)
        {
            nearest_node = i;
            nearest_distance = distance;
        }
    }
    return nearest_node;
}

// This method returns the distance between a node and a point in space
    // Args:
    //     node (Node): the node from where to measure
    //     point (std::vector<double>): the point in space
    // Returns:
    //     distance (double): the distance between node and point
double RRT::distanceNodePoint(Node node, std::vector<double> &point)
{
    double xdif = point[0] - node.x;
    double ydif = point[1] - node.y;

    return sqrt(xdif * xdif + ydif * ydif);
}

/*  Expands the tree towards the sample point (within a max dist). 
    Creates new node which is in between nearest and sample point.

    Args:
       nearest_node (Node): nearest node on the tree to the sampled point
       sampled_point (std::vector<double>): the sampled point in free space
    Returns:
       new_node (Node): new node created from steering
*/
Node RRT::steer(Node &nearest_node, std::vector<double> &sampled_point)
{
    Node new_node;
    new_node.is_root = false;

    double distance = distanceNodePoint(nearest_node, sampled_point);
    if (distance <= step_length)
    {

        new_node.x = sampled_point[0];
        new_node.y = sampled_point[1];
    }
    else
    {
        //steer towards sample point with a maximum of step_length
        new_node.x = nearest_node.x + (sampled_point[0] - nearest_node.x) * step_length / distance;
        new_node.y = nearest_node.y + (sampled_point[1] - nearest_node.y) * step_length / distance;
    }

    return new_node;
}

// This method returns a boolean indicating if the path between the
    // nearest node and the new node created from steering is collision free
    // Args:
    //    nearest_node (Node): nearest node on the tree to the sampled point
    //    new_node (Node): new node created from steering
    // Returns:
    //    collision (bool): true if in collision, false otherwise
bool RRT::check_collision(Node &nearest_node, Node &new_node)
{
    //sample in regular intervals and check if a collision occurs
    for (double i = collision_accuracy; i <= 1; i += collision_accuracy)
    {

        double x = nearest_node.x + (new_node.x - nearest_node.x) * i;
        double y = nearest_node.y + (new_node.y - nearest_node.y) * i;
        //check for collision
        if (distance_transform(x, y) == 0)
        {
            return true;
        }
    }

    return false;
}


// This method checks if the latest node added to the tree is close
// enough (defined by goal_threshold) to the goal so we can terminate
// the search and find a path
    // Args:
    //   latest_added_node (Node): latest addition to the tree
    // Returns:
    //   close_enough (bool): true if node close enough to the goal
bool RRT::is_goal(Node &latest_added_node)
{
    bool close_enough = false;
    
    double dist_goal = distanceNodePoint(latest_added_node, q_goal);
    if (dist_goal < goal_threshold)
    {
        close_enough = true;
    }

    return close_enough;
}

// This method traverses the tree from the node that has been determined
// as goal to find a path between the goal and the root.
// Args:
//   tree (sdt::vector<Node>): the rrt tree
//   latest_added_node (Node): latest addition to the tree that has been
//      determined to be close enough to the goal
// Returns:
//   path (std::vector<Node>): the vector that represents the order of
 //      of the nodes traversed as the found path
std::vector<Node> RRT::find_path(std::vector<Node> &tree, Node &latest_added_node)
{

    std::vector<Node> found_path;
    Node n = latest_added_node;
    int count = 0;
    while (!n.is_root)
    {
        if (count < rrt_steps){
            found_path.push_back(n);
            n = tree[n.parent];
            count++;
        }else
        {
            ROS_ERROR_STREAM("path counter exceeds the rrt_steps "<<rrt_steps);
            break;
        }
        
    }
    found_path.push_back(n);

    return found_path;
}

// RRT* methods--------------------------------------------------------

// This method looks if there is a node in the new_nodes neighbourhood, 
    // that has a lower cost than near, if yes the node is
    // the new parent of new_node, else near becomes new_nodes parent.
    // The nodes in the neighbourhood are also checked if they would have
    // a lower cost, when new_node where its parent and switches if so.
    // Args:
    //   tree (std::vector<Node>): the complete tree of nodes
    //   x_new (Node): the new node to be added in the tree
    //   near (int): the index of the nearest node to the samplepoint
    // Returns:
void RRT::rrt_star(std::vector<Node> &tree, Node &x_new, int &near){
    std::vector<int> neighbourhood = RRT::near(tree, x_new); 

    int min_node = near;
    x_new.old_parent = near;
    double cost_min = cost(tree[near], x_new);
    
    //check if there is a parent with lower cost in the neighbourhood
    for (int i:neighbourhood) 
    {                
        double cost_neighbour = cost(tree[i],x_new);          
        if(!check_collision(tree[i], x_new) && cost_neighbour<cost_min)
        {
            min_node = i;
            cost_min = cost_neighbour;
        }
    }

    x_new.cost = cost_min;                   
    x_new.parent = min_node;
    tree.push_back(x_new);

    //rewire the tree if there is a better path
    for (int i:neighbourhood)   
    {    
        Node *neighbour = &tree[i];
        double cost_newroute = cost(x_new,*neighbour); 
        if(!check_collision(x_new,*neighbour) && cost_newroute<(*neighbour).cost){
            if(x_new.parent==i)
            {
                ROS_ERROR_STREAM("Child and parent are the same!");
            }                     

            //make x_new the new parent of neighbour.
            (*neighbour).parent = tree.size()-1;   
        }
    }
}

// This method returns the cost associated with a node
    // Args:
    //    parent (Node): the parent of node
    //    node (Node): the node the cost is calculated for
    // Returns:
    //    cost (double): the cost value associated with the node
double RRT::cost(Node &parent, Node &node)
{      
    double cost = parent.cost + line_cost(parent,node);
    return cost;
}

// This method returns the cost of the straight line path between two nodes
    // Args:
    //    n1 (Node): the Node at one end of the path
    //    n2 (Node): the Node at the other end of the path
    // Returns:
    //    cost (double): the cost value associated with the path
double RRT::line_cost(Node &n1, Node &n2)
{
    double cost = sqrt(pow(n1.x-n2.x,2)+pow(n1.y-n2.y,2));
    return cost;
}

// This method returns the set of Nodes in the neighborhood of a node.
// Currently it has a time complexity of O(n^2), but this could be optimized
// through a search tree.
// Args:
//   tree (std::vector<Node>): the current tree
//   node (Node): the node to find the neighborhood for
// Returns:
//   neighborhood (std::vector<int>): the IDs of the nodes in the neighborhood
std::vector<int> RRT::near(std::vector<Node> &tree, Node &node)
{
    
    // radius of the neighbourhood
    //double radius = dRRT*tree.size();
    int n = tree.size();

    //to enlarge the neighbourhood a bit, +0.5.
    double radius = min(dRRT*(log(n)/n)*1/2,step_length)+0.5; 
   
    double dist;
    std::vector<int> neighborhood;

    //iterate through tree to find nodes that are close to the target
    for(int i= 0; i<tree.size(); i++){
        dist = sqrt(fabs(pow(node.x-tree[i].x,2)+pow(node.y-tree[i].y,2)));
        if(dist < radius){
            neighborhood.push_back(i);
        }
    }

    return neighborhood;
}
// Publishes the rrt tree. 
// This method is a modified version from here:https://github.com/mlab-upenn/f110_rrt_skeleton/blob/master/src/rrt.cpp
// Args:
//   tree (std::vector<Node>): the current tree
void RRT::pub_tree(std::vector<Node> &tree)
{
    // publish the current tree as a float array topic
    // published as [n1.x, n1.y, n1.parent.x, n1.parent.y, ......]
    int tree_length = tree.size();
    std_msgs::Float64MultiArray tree_msg;
    for (int i = 1; i < tree_length; i++)
    {
        double x = tree[i].x, y = tree[i].y;
        double px, py,opx,opy;
        if (tree[i].parent == -1)
        {
            px = 0.0;
            py = 0.0;
        }
        else
        {
            if(abs(tree[i].parent)<tree.size()){
                px = tree[tree[i].parent].x, py = tree[tree[i].parent].y;
            }else
            {
                ROS_ERROR_STREAM("The parent of node" <<i<<" is not in the tree. It's assigned parent is: "<<tree[i].parent);
                break;
            }
            
        }
        if (tree[i].old_parent == -1)
        {
            opx = 0.0;
            opy = 0.0;
        }
        else
        {
            if(abs(tree[i].old_parent)<tree.size()){
                opx = tree[tree[i].old_parent].x, opy = tree[tree[i].old_parent].y;
            }else
            {
                ROS_ERROR_STREAM("The parent of node" <<i<<" is not in the tree. It's assigned parent is: "<<tree[i].parent);
                break;
            }
            
        }
        tree_msg.data.push_back(x);
        tree_msg.data.push_back(y);
        //parent
        tree_msg.data.push_back(px);
        tree_msg.data.push_back(py);
        //old parent (to differentiate between rrt and rrt*)
        tree_msg.data.push_back(opx);
        tree_msg.data.push_back(opy);
    }
    tree_pub_.publish(tree_msg);
}
// This method receives the map and prepares a threshhold map from it.
// The generation of the threshold map was copied from the racecar_simulator package.
// Args: 
//     msg (nav_msgs::OccupancyGrid) the occupancy grid received
void RRT::map_callback(const nav_msgs::OccupancyGrid &msg)
{
    // Fetch the map parameters
    height = msg.info.height;
    width = msg.info.width;
    resolution = msg.info.resolution;

    // Convert the ROS origin to a pose
    origin.x = msg.info.origin.position.x;
    origin.y = msg.info.origin.position.y;
    geometry_msgs::Quaternion q = msg.info.origin.orientation;
    tf2::Quaternion quat(q.x, q.y, q.z, q.w);
    origin.theta = tf2::impl::getYaw(quat);

    // Convert the map to probability values
    std::vector<double> map(msg.data.size());
    for (size_t i = 0; i < height * width; i++)
    {
        if (msg.data[i] > 100 or msg.data[i] < 0)
        {
            map[i] = 0.5; // Unknown
        }
        else
        {
            map[i] = msg.data[i] / 100.;
        }
    }

    // Assign parameters
    origin_c = std::cos(origin.theta);
    origin_s = std::sin(origin.theta);
    double free_threshold = 0.8;
    // Threshold the map
    dt = std::vector<double>(map.size());
    for (size_t i = 0; i < map.size(); i++)
    {
        if (0 <= map[i] and map[i] <= free_threshold)
        {
            dt[i] = 99999; // Free
        }
        else
        {
            dt[i] = 0; // Occupied
        }
    }
    

    ROS_INFO_STREAM("Map initialized");
    if (!rrt_tree_built)
    {
        rrt_loop();
    }
}


// This method was copied from the racecar_simulator package. 
// It is part of the collision detection system which is used for the simulation of the lidar scan.
// We are using it for our regular collision detection and since we had to make a few minor changes,
// we copied it instead of using the original methods because we try to make as little changes as possible to the simulator.
double RRT::trace_ray(double x, double y, double theta_index) const
{
    // Add 0.5 to make this operation round rather than floor
    int theta_index_ = theta_index + 0.5;
    //double s = sines[theta_index_];
    //double c = cosines[theta_index_];
    double s = std::sin(theta_index_);
    double c = std::cos(theta_index_);

    // Initialize the distance to the nearest obstacle
    double distance_to_nearest = distance_transform(x, y);
    double total_distance = distance_to_nearest;
    double ray_tracing_epsilon = 0.0001;
    while (distance_to_nearest > ray_tracing_epsilon)
    {
        // Move in the direction of the ray
        // by distance_to_nearest
        x += distance_to_nearest * c;
        y += distance_to_nearest * s;

        // Compute the nearest distance at that point
        distance_to_nearest = distance_transform(x, y);
        total_distance += distance_to_nearest;
    }

    return total_distance;
}
// This method was copied from the racecar_simulator package. 
// It is part of the collision detection system which is used for the simulation of the lidar scan.
// We are using it for our regular collision detection and since we had to make a few minor changes,
// we copied it instead of using the original methods because we try to make as little changes as possible to the simulator.
double RRT::distance_transform(double x, double y) const
{
    // Convert the pose to a grid cell
    int cell = xy_to_cell(x, y);
    if (cell < 0)
        return 0;

    return dt[cell];
}

// This method was copied from the racecar_simulator package. 
// It is part of the collision detection system which is used for the simulation of the lidar scan.
// We are using it for our regular collision detection and since we had to make a few minor changes,
// we copied it instead of using the original methods because we try to make as little changes as possible to the simulator.
void RRT::xy_to_row_col(double x, double y, int *row, int *col) const
{
    // Translate the state by the origin
    double x_trans = x - origin.x;
    double y_trans = y - origin.y;

    // Rotate the state into the map
    double x_rot = x_trans * origin_c + y_trans * origin_s;
    double y_rot = -x_trans * origin_s + y_trans * origin_c;

    // Clip the state to be a cell
    if (x_rot < 0 or x_rot >= width * resolution or
        y_rot < 0 or y_rot >= height * resolution)
    {
        *col = -1;
        *row = -1;
    }
    else
    {
        // Discretize the state into row and column
        *col = std::floor(x_rot / resolution);
        *row = std::floor(y_rot / resolution);
    }
}

// This method was copied from the racecar_simulator package. 
// It is part of the collision detection system which is used for the simulation of the lidar scan.
// We are using it for our regular collision detection and since we had to make a few minor changes,
// we copied it instead of using the original methods because we try to make as little changes as possible to the simulator.
int RRT::row_col_to_cell(int row, int col) const
{
    return row * width + col;
}

// This method was copied from the racecar_simulator package. 
// It is part of the collision detection system which is used for the simulation of the lidar scan.
// We are using it for our regular collision detection and since we had to make a few minor changes,
// we copied it instead of using the original methods because we try to make as little changes as possible to the simulator.
int RRT::xy_to_cell(double x, double y) const
{
    int row, col;
    xy_to_row_col(x, y, &row, &col);
    return row_col_to_cell(row, col);
}
