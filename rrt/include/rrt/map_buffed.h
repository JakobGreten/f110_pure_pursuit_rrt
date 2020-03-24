#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <vector>

class MAPBUFFED {
public:
    MAPBUFFED(ros::NodeHandle &nh);
    virtual ~MAPBUFFED();
private:
    ros::NodeHandle nh_;
    ros::Publisher map_buffed_pub;

    ros::Subscriber map_sub;

    std::string map_topic, map_buffed_topic;
    void map_callback(const nav_msgs::OccupancyGrid &msg);
};