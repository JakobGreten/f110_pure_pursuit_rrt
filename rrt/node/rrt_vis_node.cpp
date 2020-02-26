// ESE 680
// RRT assignment
// Author: Hongrui Zheng

// This file contains the node definition for RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf


#include "rrt/vis_tree.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "vis_tree");
    ros::NodeHandle nh;
    RRTVIS vis_tree(nh);
    ros::spin();
    return 0;
}
