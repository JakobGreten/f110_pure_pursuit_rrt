#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <std_msgs/Float64MultiArray.h>

class PurePursuit
{
private:
    ros::NodeHandle n;
    ros::Subscriber pose_sub;
    ros::Subscriber path_sub;

    // Publish drive data
    ros::Publisher drive_pub;

    ros::Publisher vis_pub;

    double max_speed, max_steering_angle;

public:
    PurePursuit()
    {
        n = ros::NodeHandle();
        std::string drive_topic, pose_topic;
        n.getParam("pure_pursuit_node/pp_drive_topic", drive_topic);
        //n.getParam("pure_pursuit_node/pose_topic", pose_topic);
        pose_topic="/gt_pose"; //other pose topic not working
        n.getParam("pure_pursuit_node/max_speed", max_speed);
        n.getParam("pure_pursuit_node/max_steering_angle", max_steering_angle);

        drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 10);

        pose_sub = n.subscribe(pose_topic, 10, &PurePursuit::pose_callback, this);
        path_sub = n.subscribe("/path", 10, &PurePursuit::path_callback, this);
    }

    void pose_callback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg)
    {

        ackermann_msgs::AckermannDriveStamped drive_st_msg;
        ackermann_msgs::AckermannDrive drive_msg;

        /// SPEED CALCULATION:
        // set constant speed to be half of max speed
        drive_msg.speed = max_speed / 2.0;
        drive_msg.steering_angle = 0.0;
        drive_st_msg.drive = drive_msg;
        // publish AckermannDriveStamped message to drive topic
        drive_pub.publish(drive_st_msg);

        // TODO: find the current waypoint to track using methods mentioned in lecture

        // TODO: transform goal point to vehicle frame of reference

        // TODO: calculate curvature/steering angle

        // TODO: publish drive message, don't forget to limit the steering angle between -0.4189 and 0.4189 radians
    }
    void path_callback(const std_msgs::Float64MultiArray::ConstPtr &msg)
    {
    }
};
int main(int argc, char **argv)
{
    ros::init(argc, argv, "pure_pursuit_node");
    PurePursuit pp;
    ros::spin();
    return 0;
}