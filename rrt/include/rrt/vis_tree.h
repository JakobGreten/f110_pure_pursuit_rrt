#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

#include <vector>

class RRTVIS {
public:
    RRTVIS(ros::NodeHandle &nh);
    virtual ~RRTVIS();
private:
    ros::NodeHandle nh_;
    ros::Publisher pt_pub;
    ros::Publisher l_pub;
    ros::Publisher l_star_pub;
    ros::Publisher p_pub;
    ros::Publisher wpt_pub;
    ros::Publisher g_pub;

    ros::Subscriber tree_sub;
    ros::Subscriber path_sub;
    ros::Subscriber wpt_sub;

    std::string tree_topic, wpt_viz_topic, path_topic, tree_nodes, 
                rrt_lines,rrt_star_lines, path_lines, waypoint_marker, map_topic;
    void tree_callback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    void path_callback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    void wpt_callback(const geometry_msgs::Point::ConstPtr& msg);
};