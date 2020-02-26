// ESE 680
// RRT assignment
// Author: Hongrui Zheng

// This file contains the class definition of tree nodes and RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
// Make sure you have read through the header file as well
#include "rrt/rrt.h"
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
    std::string pose_topic, scan_topic;
    nh_.getParam("pose_topic", pose_topic);
    nh_.getParam("scan_topic", scan_topic);
    scan_topic="/scan";
    pose_topic="/pose";
    // ROS publishers
    // TODO: create publishers for the the drive topic, and other topics you might need
    vis_pub = nh_.advertise<visualization_msgs::Marker>("rrt_marker", 0);
    tree_pub_=nh_.advertise<std_msgs::Float64MultiArray>("/tree", 0);
    // ROS subscribers
    // TODO: create subscribers as you need
    pf_sub_ = nh_.subscribe(pose_topic, 10, &RRT::pf_callback, this);
    scan_sub_ = nh_.subscribe(scan_topic, 10, &RRT::scan_callback, this);
    ROS_INFO_STREAM("Scan topic"<<scan_topic.c_str());
    // TODO: create a occupancy grid
    gen= std::mt19937(123);
    x_dist=std::uniform_real_distribution<double>(-15.0,15.0);
    y_dist=std::uniform_real_distribution<double>(-15.0,15.0);
    
    


    //ROS_INFO(pose_topic);
    //ROS_INFO(scan_topic);
    ROS_INFO("Created new RRT Object.");

    start_visualization();

    q_goal.push_back(20);
    q_goal.push_back(20);

    step_length=0.1;
    
    
    Node start;
    start.x=0;
    start.y=0;
    start.cost=1;
    start.is_root=true;
    tree.push_back(start);
    
    int counter = 0;
    while(counter<5000){
        std::vector<double> sampled_point=sample();
        ROS_INFO_STREAM(sampled_point[0]);
        int near=nearest(tree,sampled_point);
        Node x_new = steer(near,tree[near],sampled_point);
        tree.push_back(x_new);
        counter++;
    }

}

void RRT::start_visualization()
{


    // TODO: fill in the RRT main loop
    /*Node start;
    start.x=0;
    start.y=0;
    start.cost=1;
    start.is_root=true;
    tree.push_back(start);
    Node two;
    two.x=7;
    two.y=5;
    two.cost=1;
    two.is_root=false;
    two.parent=0;
    tree.push_back(two);

    Node three;
    three.x=20;
    three.y=8;
    three.cost=1;
    three.is_root=false;
    three.parent=0;
    tree.push_back(three);
    pub_tree(tree);*/


   
}

void RRT::scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    pub_tree(tree);
    //ROS_INFO("scan");

    // The scan callback, update your occupancy grid here
    // Args:
    //    scan_msg (*LaserScan): pointer to the incoming scan message
    // Returns:
    //

    // TODO: update your occupancy grid
}

void RRT::pf_callback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg)
{
    // The pose callback when subscribed to particle filter's inferred pose
    // The RRT main loop happens here
    // Args:
    //    pose_msg (*PoseStamped): pointer to the incoming pose message
    // Returns:
    //

    // tree as std::vector
    std::vector<Node> tree;

    // TODO: fill in the RRT main loop

    // path found as Path message
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




    // TODO: fill in this method
    // look up the documentation on how to use std::mt19937 devices with a distribution
    // the generator and the distribution is created for you (check the header file)

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
    double nearest_distance=9999999;
    for (int i=0; i < tree.size(); i++) {
       

       double distance = distanceNodePoint(tree[i],sampled_point);
       if(distance<nearest_distance){
           nearest_node=i;
           nearest_distance=distance;
       }

    }
    
    
    // TODO: fill in this method

    return nearest_node;
}
double RRT::distanceNodePoint(Node node,std::vector<double> &point){
    double xdif = point[0]-node.x;
    double ydif = point[1]-node.y;

    return sqrt(xdif*xdif+ydif*ydif);
}


Node RRT::steer(int parent,Node &nearest_node, std::vector<double> &sampled_point)
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
    new_node.parent=parent;
    new_node.is_root=false;
    new_node.cost=1;


    double distance = distanceNodePoint(nearest_node,sampled_point);
    ROS_INFO_STREAM("Distance"<<distance);
    if(distance<=step_length){
    
        new_node.x=sampled_point[0];
        new_node.y=sampled_point[1];
    
    
    }else{
        new_node.x=nearest_node.x+(sampled_point[0]-nearest_node.x)*step_length/distance;
        new_node.y=nearest_node.y+(sampled_point[1]-nearest_node.y)*step_length/distance;
        ROS_INFO("step");

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

    bool collision = false;
    // TODO: fill in this method

    return collision;
}

bool RRT::is_goal(Node &latest_added_node, double goal_x, double goal_y)
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
    // TODO: fill in this method

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
void RRT::pub_tree(std::vector<Node> &tree) {
    // publish the current tree as a float array topic
    // published as [n1.x, n1.y, n1.parent.x, n1.parent.y, ......]
    int tree_length = tree.size();
    std_msgs::Float64MultiArray tree_msg;
    for (int i=1; i<tree_length; i++) {
        double x = tree[i].x, y = tree[i].y;
        double px, py;
        if (tree[i].parent == -1) {
            px = 0.0;
            py = 0.0;
        } else {
            px = tree[tree[i].parent].x, py = tree[tree[i].parent].y;
        }
        tree_msg.data.push_back(x);
        tree_msg.data.push_back(y);
        tree_msg.data.push_back(px);
        tree_msg.data.push_back(py);
    }
    tree_pub_.publish(tree_msg);
}