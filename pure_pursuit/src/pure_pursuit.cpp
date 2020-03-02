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
    int sphere_marker_idx, cylinder_marker_idx;
    std::vector<double> path;

public:
    PurePursuit()
    {
        n = ros::NodeHandle();
        std::string drive_topic, pose_topic;
        n.getParam("pure_pursuit_node/pp_drive_topic", drive_topic);
        //n.getParam("pure_pursuit_node/pose_topic", pose_topic);
        pose_topic = "/gt_pose"; //other pose topic not working
        n.getParam("pure_pursuit_node/max_speed", max_speed);
        n.getParam("pure_pursuit_node/max_steering_angle", max_steering_angle);

        drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 10);
        vis_pub = n.advertise<visualization_msgs::Marker>("pure_pursuit_marker", 10);

        pose_sub = n.subscribe(pose_topic, 10, &PurePursuit::pose_callback, this);
        path_sub = n.subscribe("/path", 10, &PurePursuit::path_callback, this);
    }

    void pose_callback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg)
    {
        sphere_marker_idx=0;
        cylinder_marker_idx=0;
        ackermann_msgs::AckermannDriveStamped drive_st_msg;
        ackermann_msgs::AckermannDrive drive_msg;

        /// SPEED CALCULATION:
        // set constant speed to be half of max speed
        drive_msg.speed = max_speed / 2.0;
        drive_msg.steering_angle = 0.0;
        drive_st_msg.drive = drive_msg;
        // publish AckermannDriveStamped message to drive topic
        drive_pub.publish(drive_st_msg);
        //publishCylinder(1,1,0.7);
        //publishSphere(2,2);
        //publishSphere(3,3);
        //publishCylinder(-1,-1,0.2);

        // TODO: find the current waypoint to track using methods mentioned in lecture

        // TODO: transform goal point to vehicle frame of reference

        // TODO: calculate curvature/steering angle

        // TODO: publish drive message, don't forget to limit the steering angle between -0.4189 and 0.4189 radians
    }
    void path_callback(const std_msgs::Float64MultiArray::ConstPtr &msg)
    {
        path = msg->data;
    }
    double distance(double ax, double ay, double bx, double by)
    {
        double xdif = ax - bx;
        double ydif = ay - by;
        return sqrt(xdif * xdif + ydif * ydif);
    }
    
    void publishSphere(double x, double y)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/map";
        marker.header.stamp = ros::Time::now();

        marker.ns = "sphere";
        marker.id = sphere_marker_idx;
        sphere_marker_idx++;

        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        double size=0.1;
        marker.scale.x = size;
        marker.scale.y = size;
        marker.scale.z = size;

        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration();
        vis_pub.publish(marker);
    }
    void publishCylinder(double x, double y,double diameter)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/map";
        marker.header.stamp = ros::Time::now();

        marker.ns = "cylinder";
        marker.id = cylinder_marker_idx;
        cylinder_marker_idx++;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = diameter;
        marker.scale.y = diameter;
        marker.scale.z = 0.1;

        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration();
        vis_pub.publish(marker);
    }
};
int main(int argc, char **argv)
{
    ros::init(argc, argv, "pure_pursuit_node");
    PurePursuit pp;
    ros::spin();
    return 0;
}