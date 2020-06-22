// ESE 680
// RRT assignment
// Author: Hongrui Zheng

// This file contains the class definition of tree nodes and RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf

// ros

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_listener.h>
#include <tf2/impl/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
//#include "rrt/pose_2d.hpp"
#include "rrt/distance_transform.hpp"
// standard
#include <math.h>
#include <vector>
#include <array>
#include <iostream>
#include <fstream>
#include <iterator>
#include <string>
#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <random>

#include <visualization_msgs/Marker.h>

// Struct defining the Node object in the RRT tree.
// More fields could be added to thiis struct if more info needed.
// You can choose to use this or not
typedef struct Node
{
    double x, y;
    double cost; // only used for RRT*
    int parent, old_parent; // index of parent node in the tree vector
    bool is_root = false;
} Node;

struct Pose2D
{
    double x;
    double y;
    double theta;
};

class RRT
{
public:
    RRT(ros::NodeHandle &nh);
    virtual ~RRT();
    std::vector<double> sines;
    std::vector<double> cosines;

private:
    ros::NodeHandle nh_;

    // ros pub/sub
    // TODO: add the publishers and subscribers you need
    ros::Publisher vis_pub;
    ros::Publisher tree_pub_;
    ros::Publisher path_pub_;

    ros::Subscriber pf_sub_;
    ros::Subscriber scan_sub_;
    ros::Subscriber map_sub;
    ros::Subscriber click_sub;
    ros::Subscriber nav_goal_sub;

    // tf stuff
    tf::TransformListener listener;

    // TODO: create RRT params

    // random generator, use this
    std::mt19937 gen;
    std::uniform_real_distribution<double> x_dist;
    std::uniform_real_distribution<double> y_dist;

    visualization_msgs::Marker points, line_strip, line_list;

    std::vector<Node> tree;
    std::vector<Node> path;

    double pose_x, pose_y, pose_theta;

    std::vector<double> q_goal;
    double goal_threshold;
    double dRRT;
    double step_length;
    double rrt_bias;
    int rrt_steps;
    int max_rrt_iterations;
    double collision_accuracy;
    bool rrt_tree_build;
    std::string pose_topic, scan_topic, path_topic, clicked_point_topic, 
                map_topic, nav_goal_topic, marker_topic, tree_topic;

    // The distance transform
    double resolution;
    size_t width, height;
    Pose2D origin;
    std::vector<double> dt;
    double origin_c;
    double origin_s;

    // callbacks
    // where rrt actually happens
    void pf_callback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg);
    // updates occupancy grid
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg);
    void clicked_point_callback(const geometry_msgs::PointStamped &pose_msg);
    void nav_goal_callback(const geometry_msgs::PoseStamped &pose_msg);


    double distanceNodePoint(Node node, std::vector<double> &point);
    void rrt_loop();

    // RRT methods
    std::vector<double> sample();
    int nearest(std::vector<Node> &tree, std::vector<double> &sampled_point);
    Node steer(Node &nearest_node, std::vector<double> &sampled_point);
    bool check_collision(Node &nearest_node, Node &new_node);
    bool is_goal(Node &latest_added_node);
    std::vector<Node> find_path(std::vector<Node> &tree, Node &latest_added_node);
    
    // RRT* methods
    void rrt_star(std::vector<Node> &tree, Node &node, int &near);
    double cost(Node &parent, Node &node);
    double line_cost(Node &n1, Node &n2);
    std::vector<int> near(std::vector<Node> &tree, Node &node);

    void pub_tree(std::vector<Node> &tree);
    void map_callback(const nav_msgs::OccupancyGrid &msg);
    void xy_to_row_col(double x, double y, int *row, int *col) const;
    int row_col_to_cell(int row, int col) const;
    int xy_to_cell(double x, double y) const;
    double distance_transform(double x, double y) const;
    double trace_ray(double x, double y, double theta_index) const;
};
