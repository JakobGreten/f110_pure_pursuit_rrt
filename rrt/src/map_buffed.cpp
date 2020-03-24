#include "rrt/map_buffed.h"

MAPBUFFED::~MAPBUFFED() {
    ROS_INFO("RRT_VIS shutting down");
}

MAPBUFFED::MAPBUFFED(ros::NodeHandle &nh) : nh_(nh) {
    nh_.getParam("rrt/map_topic", map_topic);
    nh_.getParam("rrt/map_buffed_topic", map_buffed_topic);

    // tree_topic="/tree";
    // wpt_viz_topic="/wpt_viz";
    
    map_buffed_pub = nh_.advertise<nav_msgs::OccupancyGrid>(map_buffed_topic, 10);

    map_sub = nh_.subscribe(map_topic, 10, &MAPBUFFED::map_callback, this);
}

void MAPBUFFED::map_callback(const nav_msgs::OccupancyGrid &msg) {
    nav_msgs::OccupancyGrid oGrid;
    map_buffed_pub.publish(oGrid);

}


/*int main(int argc, char** argv) {
    ros::init(argc, argv, "vis_tree");
    ros::NodeHandle nh;
    MAPBUFFED map_buffed(nh);
    ros::spin();
    return 0;
}*/
