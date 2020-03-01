#include "rrt/vis_tree.h"

RRTVIS::~RRTVIS() {
    ROS_INFO("RRT_VIS shutting down");
}

RRTVIS::RRTVIS(ros::NodeHandle &nh) : nh_(nh) {
    std::string tree_topic, wpt_viz_topic,path_topic;
    nh_.getParam("tree_topic", tree_topic);
    nh_.getParam("rrt/path_topic", path_topic);
    nh_.getParam("waypoint_viz_topic", wpt_viz_topic);
    tree_topic="/tree";
    wpt_viz_topic="/wpt_viz";
    pt_pub = nh_.advertise<visualization_msgs::Marker>("tree_nodes", 10);
    l_pub = nh_.advertise<visualization_msgs::Marker>("tree_lines", 10);
    p_pub = nh_.advertise<visualization_msgs::Marker>("path_lines", 10);
    wpt_pub = nh_.advertise<visualization_msgs::Marker>("waypoint_marker", 10);

    tree_sub = nh_.subscribe(tree_topic, 10, &RRTVIS::tree_callback, this);
    path_sub = nh_.subscribe(path_topic, 10, &RRTVIS::path_callback, this);
    //wpt_sub = nh_.subscribe(wpt_viz_topic, 10, &RRTVIS::wpt_callback, this);
}

void RRTVIS::tree_callback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.type = marker.LINE_LIST;
    marker.scale.x = 0.015;
    marker.scale.y = 0.015;
    marker.scale.z = 0.015;

    visualization_msgs::Marker node_marker;
    node_marker.header.frame_id = "/map";
    node_marker.type = node_marker.SPHERE_LIST;
    node_marker.scale.x = 0.05;
    node_marker.scale.y = 0.05;
    node_marker.scale.z = 0.05;

    std_msgs::ColorRGBA col;
    col.a = 1.0;
    col.r = 0.0;
    col.g = 0.0;
    col.b = 1.0;

    std_msgs::ColorRGBA node_col;
    node_col.a = 1.0;
    node_col.r = 1.0;
    node_col.g = 0.0;
    node_col.b = 1.0;

    for (int i=0; i<msg->data.size()/4; i++) {
        double x = msg->data[i*4];
        double y = msg->data[i*4+1];
        double px = msg->data[i*4+2];
        double py = msg->data[i*4+3];
        geometry_msgs::Point point;
        point.x = x;
        point.y = y;
        point.z = 0.0;
        geometry_msgs::Point parent_point;
        parent_point.x = px;
        parent_point.y = py;
        parent_point.z = 0.0;
        marker.points.push_back(point);
        marker.points.push_back(parent_point);
        marker.colors.push_back(col);
        marker.colors.push_back(col);

        node_marker.points.push_back(point);
        node_marker.colors.push_back(node_col);
    }
    pt_pub.publish(node_marker);
    l_pub.publish(marker);
}

void RRTVIS::path_callback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.type = marker.LINE_STRIP;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    std_msgs::ColorRGBA col;
    col.a = 1.0;
    col.r = 1.0;
    col.g = 0.0;
    col.b = 0.0;

    for (int i=0; i<msg->data.size()/2; i++) {
        double x = msg->data[i*2];
        double y = msg->data[i*2+1];
        geometry_msgs::Point point;
        point.x = x;
        point.y = y;
        point.z = 0.0;
        marker.points.push_back(point);
        marker.colors.push_back(col);
    }

    p_pub.publish(marker);
}



void RRTVIS::wpt_callback(const geometry_msgs::Point::ConstPtr& msg) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
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
