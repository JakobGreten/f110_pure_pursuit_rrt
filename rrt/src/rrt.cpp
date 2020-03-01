// ESE 680
// RRT assignment
// Author: Hongrui Zheng

// This file contains the class definition of tree nodes and RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
// Make sure you have read through the header file as well
#include "rrt/rrt.h"
//#include "rrt/pose_2d.hpp"

#include <random>

// Destructor of the RRT class
RRT::~RRT()
{
    // Do something in here, free up used memory, print message, etc.
    ROS_INFO("RRT shutting down");
}

// Constructor of the RRT class
RRT::RRT(ros::NodeHandle &nh) : nh_(nh), gen((std::random_device())())
{

    // TODO: Load parameters from yaml file, you could add your own parameters to the rrt_params.yaml file
    std::string pose_topic, scan_topic, path_topic;
    nh_.getParam("rrt/pose_topic", pose_topic);
    nh_.getParam("rrt/scan_topic", scan_topic);
    nh_.getParam("rrt/path_topic", path_topic);

    nh_.getParam("rrt/rrt_steps", rrt_steps);
    nh_.getParam("rrt/collision_accuracy", collision_accuracy);
    nh_.getParam("rrt/step_length", step_length);
    nh_.getParam("rrt/goal_threshold", goal_threshold);

    ROS_INFO_STREAM("rrt_steps" << rrt_steps << " pose topic " << pose_topic);
    // ROS publishers
    // TODO: create publishers for the the drive topic, and other topics you might need
    vis_pub = nh_.advertise<visualization_msgs::Marker>("rrt_marker", 0);
    tree_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/tree", 0);
    path_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(path_topic, 10);
    // ROS subscribers
    // TODO: create subscribers as you need
    pf_sub_ = nh_.subscribe(pose_topic, 10, &RRT::pf_callback, this);
    scan_sub_ = nh_.subscribe(scan_topic, 10, &RRT::scan_callback, this);
    map_sub = nh_.subscribe("/map", 1, &RRT::map_callback, this);
    click_sub = nh_.subscribe("/clicked_point", 1, &RRT::clicked_point_callback, this);
    nav_goal_sub = nh_.subscribe("/move_base_simple/goal", 1, &RRT::nav_goal_callback, this);

    //ROS_INFO_STREAM("Scan topic"<<scan_topic.c_str());
    // TODO: create a occupancy grid
    gen = std::mt19937(123);
    x_dist = std::uniform_real_distribution<double>(-15.0, 15.0);
    y_dist = std::uniform_real_distribution<double>(-15.0, 15.0);

    //ROS_INFO(pose_topic);
    //ROS_INFO(scan_topic);
    rrt_tree_build = false;
    ROS_INFO("Created new RRT Object.");
}

void RRT::rrt_loop()
{
    q_goal.push_back(8);
    q_goal.push_back(0);

    Node start;
    start.x = 0;
    start.y = 0;
    start.cost = 1;
    start.is_root = true;
    tree.push_back(start);

    int counter = 0;
    while (counter < rrt_steps)
    {
        std::vector<double> sampled_point = sample();
        //ROS_INFO_STREAM(sampled_point[0]);
        int near = nearest(tree, sampled_point);
        Node x_new = steer(near, tree[near], sampled_point);
        if (!check_collision(tree[near], x_new))
        {
            tree.push_back(x_new);
        }
        if (is_goal(x_new))
        {
            ROS_INFO_STREAM("Goal found");
            path = find_path(tree, x_new);
            break;
        }
        counter++;
    }
    rrt_tree_build = true;
}

void RRT::scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    // The scan callback, update your occupancy grid here
    // Args:
    //    scan_msg (*LaserScan): pointer to the incoming scan message
    // Returns:
    std_msgs::Float64MultiArray path_msg;
    for (int i = 0; i < path.size(); i++)
    {
        
        path_msg.data.push_back(path[i].x);
        path_msg.data.push_back(path[i].y);
    }
    path_pub_.publish(path_msg);
    pub_tree(tree);

    

}
//not being called currently
void RRT::pf_callback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg)
{
    // The pose callback when subscribed to particle filter's inferred pose
    // The RRT main loop happens here
    // Args:
    //    pose_msg (*PoseStamped): pointer to the incoming pose message
    // Returns:
    //
    //double distance_to_nearest = distance_transform(pose_msg->pose.position.x, pose_msg->pose.position.y);
    //ROS_INFO_STREAM("Distance: " << distance_to_nearest);
    //double distance_wall = trace_ray(pose_msg->pose.position.x, pose_msg->pose.position.y,0);
    //ROS_INFO_STREAM("Distance: " << distance_wall);

    // tree as std::vector
    std::vector<Node> tree;

    // TODO: fill in the RRT main loop

    // path found as Path message
}
void RRT::clicked_point_callback(const geometry_msgs::PointStamped &pose_msg)
{

    double distance_to_nearest = distance_transform(pose_msg.point.x, pose_msg.point.y);
    ROS_INFO_STREAM("Distance: " << distance_to_nearest);
    //double distance_wall = trace_ray(pose_msg->pose.position.x, pose_msg->pose.position.y,0);
    //ROS_INFO_STREAM("Distance: " << distance_wall);
}
void RRT::nav_goal_callback(const geometry_msgs::PoseStamped &pose_msg)
{
    double x = pose_msg.pose.position.x;
    double y = pose_msg.pose.position.y;
    q_goal.clear();
    q_goal.push_back(x);
    q_goal.push_back(y);
    ROS_INFO_STREAM("New nav goal set to X: " << x<<" Y: "<<y);
    tree.clear();
    path.clear();
    rrt_loop();
    
}

std::vector<double> RRT::sample()
{
    // This method returns a sampled point from the free space
    // You should restrict so that it only samples a small region
    // of interest around the car's current position
    // Args:
    // Returns:
    //     sampled_point (std::vector<double>): the sampled point in free space

    std::vector<double> sampled_point;
    sampled_point.push_back(x_dist(gen));
    sampled_point.push_back(y_dist(gen));

    return sampled_point;
}

int RRT::nearest(std::vector<Node> &tree, std::vector<double> &sampled_point)
{
    // This method returns the nearest node on the tree to the sampled point
    // Args:
    //     tree (std::vector<Node>): the current RRT tree
    //     sampled_point (std::vector<double>): the sampled point in free space
    // Returns:
    //     nearest_node (int): index of nearest node on the tree
    int nearest_node = 0;
    double nearest_distance = 9999999;
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
double RRT::distanceNodePoint(Node node, std::vector<double> &point)
{
    double xdif = point[0] - node.x;
    double ydif = point[1] - node.y;

    return sqrt(xdif * xdif + ydif * ydif);
}

Node RRT::steer(int parent, Node &nearest_node, std::vector<double> &sampled_point)
{
    // The function steer:(x,y)->z returns a point such that z is “closer”
    // to y than x is. The point z returned by the function steer will be
    // such that z minimizes ||z−y|| while at the same time maintaining
    //||z−x|| <= max_expansion_dist, for a prespecified max_expansion_dist > 0

    // basically, expand the tree towards the sample point (within a max dist)

    // Args:
    //    nearest_node (Node): nearest node on the tree to the sampled point
    //    sampled_point (std::vector<double>): the sampled point in free space
    // Returns:
    //    new_node (Node): new node created from steering

    Node new_node;
    new_node.parent = parent;
    new_node.is_root = false;
    new_node.cost = 0.6;

    double distance = distanceNodePoint(nearest_node, sampled_point);
    //ROS_INFO_STREAM("Distance"<<distance);
    if (distance <= step_length)
    {

        new_node.x = sampled_point[0];
        new_node.y = sampled_point[1];
    }
    else
    {
        new_node.x = nearest_node.x + (sampled_point[0] - nearest_node.x) * step_length / distance;
        new_node.y = nearest_node.y + (sampled_point[1] - nearest_node.y) * step_length / distance;
        //ROS_INFO("step");
    }
    // TODO: fill in this method

    return new_node;
}

bool RRT::check_collision(Node &nearest_node, Node &new_node)
{
    // This method returns a boolean indicating if the path between the
    // nearest node and the new node created from steering is collision free
    // Args:
    //    nearest_node (Node): nearest node on the tree to the sampled point
    //    new_node (Node): new node created from steering
    // Returns:
    //    collision (bool): true if in collision, false otherwise

    for (double i = collision_accuracy; i <= 1; i += collision_accuracy)
    {

        double x = nearest_node.x + (new_node.x - nearest_node.x) * i;
        double y = nearest_node.y + (new_node.y - nearest_node.y) * i;
        if (distance_transform(x, y) == 0)
        {
            return true;
        }
    }

    return false;
}

bool RRT::is_goal(Node &latest_added_node)
{
    // This method checks if the latest node added to the tree is close
    // enough (defined by goal_threshold) to the goal so we can terminate
    // the search and find a path
    // Args:
    //   latest_added_node (Node): latest addition to the tree
    //   goal_x (double): x coordinate of the current goal
    //   goal_y (double): y coordinate of the current goal
    // Returns:
    //   close_enough (bool): true if node close enough to the goal
    bool close_enough = false;

    double dist_goal = distanceNodePoint(latest_added_node, q_goal);
    if (dist_goal < goal_threshold)
    {
        close_enough = true;
    }
    // TODO: fill in this method

    return close_enough;
}

std::vector<Node> RRT::find_path(std::vector<Node> &tree, Node &latest_added_node)
{
    // This method traverses the tree from the node that has been determined
    // as goal
    // Args:
    //   latest_added_node (Node): latest addition to the tree that has been
    //      determined to be close enough to the goal
    // Returns:
    //   path (std::vector<Node>): the vector that represents the order of
    //      of the nodes traversed as the found path

    std::vector<Node> found_path;
    Node n = latest_added_node;
    while (!n.is_root)
    {
        found_path.push_back(n);
        n = tree[n.parent];
    }
    found_path.push_back(n);

    return found_path;
}

// RRT* methods
double RRT::cost(std::vector<Node> &tree, Node &node)
{
    // This method returns the cost associated with a node
    // Args:
    //    tree (std::vector<Node>): the current tree
    //    node (Node): the node the cost is calculated for
    // Returns:
    //    cost (double): the cost value associated with the node

    double cost = 0;
    // TODO: fill in this method

    return cost;
}

double RRT::line_cost(Node &n1, Node &n2)
{
    // This method returns the cost of the straight line path between two nodes
    // Args:
    //    n1 (Node): the Node at one end of the path
    //    n2 (Node): the Node at the other end of the path
    // Returns:
    //    cost (double): the cost value associated with the path

    double cost = 0;
    // TODO: fill in this method

    return cost;
}

std::vector<int> RRT::near(std::vector<Node> &tree, Node &node)
{
    // This method returns the set of Nodes in the neighborhood of a
    // node.
    // Args:
    //   tree (std::vector<Node>): the current tree
    //   node (Node): the node to find the neighborhood for
    // Returns:
    //   neighborhood (std::vector<int>): the index of the nodes in the neighborhood

    std::vector<int> neighborhood;
    // TODO:: fill in this method

    return neighborhood;
}
// For visualization
void RRT::pub_tree(std::vector<Node> &tree)
{
    // publish the current tree as a float array topic
    // published as [n1.x, n1.y, n1.parent.x, n1.parent.y, ......]
    int tree_length = tree.size();
    std_msgs::Float64MultiArray tree_msg;
    for (int i = 1; i < tree_length; i++)
    {
        double x = tree[i].x, y = tree[i].y;
        double px, py;
        if (tree[i].parent == -1)
        {
            px = 0.0;
            py = 0.0;
        }
        else
        {
            px = tree[tree[i].parent].x, py = tree[tree[i].parent].y;
        }
        tree_msg.data.push_back(x);
        tree_msg.data.push_back(y);
        tree_msg.data.push_back(px);
        tree_msg.data.push_back(py);
    }
    tree_pub_.publish(tree_msg);
}
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
    //DistanceTransform::distance_2d(dt, width, height, resolution);

    /*// Send the map to the scanner
            scan_simulator.set_map(
                map,
                height,
                width,
                resolution,
                origin,
                map_free_threshold);
            map_exists = true;*/

    ROS_INFO_STREAM("Map initialized");
    if (!rrt_tree_build)
    {
        rrt_loop();
    }
}

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

double RRT::distance_transform(double x, double y) const
{
    // Convert the pose to a grid cell
    int cell = xy_to_cell(x, y);
    if (cell < 0)
        return 0;

    return dt[cell];
}

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

int RRT::row_col_to_cell(int row, int col) const
{
    return row * width + col;
}

int RRT::xy_to_cell(double x, double y) const
{
    int row, col;
    xy_to_row_col(x, y, &row, &col);
    return row_col_to_cell(row, col);
}