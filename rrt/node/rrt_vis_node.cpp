#include "rrt/vis_tree.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "vis_tree");
    ros::NodeHandle nh;
    RRTVIS vis_tree(nh);
    ros::spin();
    return 0;
}
