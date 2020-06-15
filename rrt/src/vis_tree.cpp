// This file was copied from https://github.com/mlab-upenn/f110_rrt_skeleton
// We made some changes to the vizalization to allow for the distinction and display of both 
// a normal rrt and a rrt* tree


#include "rrt/vis_tree.h"

RRTVIS::~RRTVIS()
{
    ROS_INFO("RRT_VIS shutting down");
}

RRTVIS::RRTVIS(ros::NodeHandle &nh) : nh_(nh)
{
    nh_.getParam("rrt/tree_topic", tree_topic);
    nh_.getParam("rrt/path_topic", path_topic);
    nh_.getParam("rrt/map_topic", map_topic);
    nh_.getParam("rrt/waypoint_viz_topic", wpt_viz_topic);
    nh_.getParam("rrt/tree_nodes", tree_nodes);
    nh_.getParam("rrt/rrt_lines", rrt_lines);
    nh_.getParam("rrt/rrt_star_lines", rrt_star_lines);
    nh_.getParam("rrt/path_lines", path_lines);
    nh_.getParam("rrt/waypoint_marker", waypoint_marker);
    // tree_topic="/tree";
    // wpt_viz_topic="/wpt_viz";
    pt_pub = nh_.advertise<visualization_msgs::Marker>(tree_nodes, 10);
    l_pub = nh_.advertise<visualization_msgs::Marker>(rrt_lines, 10);
    l_star_pub = nh_.advertise<visualization_msgs::Marker>(rrt_star_lines, 10);
    p_pub = nh_.advertise<visualization_msgs::Marker>(path_lines, 10);
    wpt_pub = nh_.advertise<visualization_msgs::Marker>(waypoint_marker, 10);

    tree_sub = nh_.subscribe(tree_topic, 10, &RRTVIS::tree_callback, this);
    path_sub = nh_.subscribe(path_topic, 10, &RRTVIS::path_callback, this);
    //wpt_sub = nh_.subscribe(wpt_viz_topic, 10, &RRTVIS::wpt_callback, this);
}

void RRTVIS::tree_callback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
    visualization_msgs::Marker rrt_marker;
    rrt_marker.header.frame_id = map_topic;
    rrt_marker.type = rrt_marker.LINE_LIST;
    rrt_marker.scale.x = 0.015;
    rrt_marker.scale.y = 0.015;
    rrt_marker.scale.z = 0.015;

    rrt_marker.color.a = 1.0;
    rrt_marker.color.r = 0.0;
    rrt_marker.color.g = 0.0;
    rrt_marker.color.b = 1.0;


    visualization_msgs::Marker rrt_star_marker;
    rrt_star_marker.header.frame_id = map_topic;
    rrt_star_marker.type = rrt_star_marker.LINE_LIST;
    rrt_star_marker.scale.x = 0.040;
    rrt_star_marker.scale.y = 0.015;
    rrt_star_marker.scale.z = 0.015;

    rrt_star_marker.color.a = 1.0;
    rrt_star_marker.color.r = 0.0;
    rrt_star_marker.color.g = 0.9;
    rrt_star_marker.color.b = 0.0;


    visualization_msgs::Marker node_marker;
    node_marker.header.frame_id = map_topic;
    node_marker.type = node_marker.SPHERE_LIST;
    node_marker.scale.x = 0.05;
    node_marker.scale.y = 0.05;
    node_marker.scale.z = 0.05;

    

    std_msgs::ColorRGBA node_col;
    node_col.a = 1.0;
    node_col.r = 1.0;
    node_col.g = 0.0;
    node_col.b = 1.0;

    for (int i = 0; i < msg->data.size() / 6; i++)
    {
        double x = msg->data[i * 6];
        double y = msg->data[i * 6 + 1];
        double px = msg->data[i * 6 + 2];
        double py = msg->data[i * 6 + 3];
        double opx = msg->data[i * 6 + 4];
        double opy = msg->data[i * 6 + 5];
        //ROS_INFO_STREAM("px"<<px<<" opy"<<opy);

        geometry_msgs::Point point;
        point.x = x;
        point.y = y;
        point.z = 0.0;
        geometry_msgs::Point old_parent_point;
        old_parent_point.x = opx;
        old_parent_point.y = opy;
        old_parent_point.z = 0.0;

        if (px != opx || py != opy)
        {
            geometry_msgs::Point parent_point;
            parent_point.x = px;
            parent_point.y = py;
            parent_point.z = 0.0;

            rrt_star_marker.points.push_back(point);
            rrt_star_marker.points.push_back(parent_point);
           
        }

        rrt_marker.points.push_back(point);
        rrt_marker.points.push_back(old_parent_point);

        node_marker.points.push_back(point);
        node_marker.colors.push_back(node_col);
    }
    pt_pub.publish(node_marker);
    l_pub.publish(rrt_marker);
    l_star_pub.publish(rrt_star_marker);
}

void RRTVIS::path_callback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = map_topic;
    marker.type = marker.LINE_STRIP;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    std_msgs::ColorRGBA col;
    col.a = 1.0;
    col.r = 1.0;
    col.g = 0.0;
    col.b = 0.0;

    for (int i = 0; i < msg->data.size() / 2; i++)
    {
        double x = msg->data[i * 2];
        double y = msg->data[i * 2 + 1];
        geometry_msgs::Point point;
        point.x = x;
        point.y = y;
        point.z = 0.0;
        marker.points.push_back(point);
        marker.colors.push_back(col);
    }

    p_pub.publish(marker);
}

void RRTVIS::wpt_callback(const geometry_msgs::Point::ConstPtr &msg)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = map_topic;
    marker.type = marker.SPHERE;
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.pose.position.x = msg->x;
    marker.pose.position.y = msg->y;
    marker.pose.position.z = 0.0;
    wpt_pub.publish(marker);
}

/*int main(int argc, char** argv) {
    ros::init(argc, argv, "vis_tree");
    ros::NodeHandle nh;
    RRTVIS rrt_vis(nh);
    ros::spin();
    return 0;
}*/
