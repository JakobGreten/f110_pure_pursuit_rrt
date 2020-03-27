#include "rrt/map_buffed.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "map_buffed");
    ros::NodeHandle nh;
    MAPBUFFED map_buffed(nh);
    ros::spin();
    return 0;
}
