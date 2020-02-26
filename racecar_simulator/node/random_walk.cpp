#include <ros/ros.h>

// Publish to a topic with this message type
#include <ackermann_msgs/AckermannDriveStamped.h>
// AckermannDriveStamped messages include this message type
#include <ackermann_msgs/AckermannDrive.h>

// Subscribe to a topic with this message type
#include <nav_msgs/Odometry.h>

#include <visualization_msgs/Marker.h>
// for printing
#include <iostream>

// for RAND_MAX
#include <cstdlib>

#include <cmath>

class RandomWalker
{
private:
    // A ROS node
    ros::NodeHandle n;

    // car parameters
    double max_speed, max_steering_angle;

    // Listen for odom messages
    ros::Subscriber odom_sub;

    // Publish drive data
    ros::Publisher drive_pub;

    ros::Publisher vis_pub;
    // previous desired steering angle
    double prev_angle = 0.0;

    visualization_msgs::Marker points, line_strip, line_list;

public:
    RandomWalker()
    {
        // Initialize the node handle
        n = ros::NodeHandle("~");

        // get topic names
        std::string drive_topic, odom_topic;
        n.getParam("rand_drive_topic", drive_topic);
        n.getParam("odom_topic", odom_topic);

        // get car parameters
        n.getParam("max_speed", max_speed);
        n.getParam("max_steering_angle", max_steering_angle);

        // Make a publisher for drive messages
        drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 10);

        // Start a subscriber to listen to odom messages
        odom_sub = n.subscribe(odom_topic, 1, &RandomWalker::odom_callback, this);

        vis_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 0);

        float f = 1.5;
        points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "map";
        points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time();
        points.ns = line_strip.ns = line_list.ns = "points_and_lines";
        points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;

        points.id = 0;
        line_strip.id = 1;
        line_list.id = 2;

        points.type = visualization_msgs::Marker::POINTS;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        line_list.type = visualization_msgs::Marker::LINE_LIST;

        // POINTS markers use x and y scale for width/height respectively
        points.scale.x = 0.2;
        points.scale.y = 0.2;

        // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
        line_strip.scale.x = 0.1;
        line_list.scale.x = 0.1;

        // Points are green
        points.color.g = 1.0f;
        points.color.a = 1.0;

        // Line strip is blue
        line_strip.color.b = 1.0;
        line_strip.color.a = 1.0;

        // Line list is red
        line_list.color.r = 1.0;
        line_list.color.a = 1.0;

        // Create the vertices for the points and lines
        /*for (uint32_t i = 0; i < 100; ++i)
        {
            float y = 5 * sin(f + i / 100.0f * 2 * M_PI);
            float z = 5 * cos(f + i / 100.0f * 2 * M_PI);

            geometry_msgs::Point p;
            p.x = (int32_t)i - 50;
            p.y = y;
            p.z = z;

            points.points.push_back(p);
            line_strip.points.push_back(p);

            // The line list needs two points for each line
            line_list.points.push_back(p);
            p.z += 1.0;
            line_list.points.push_back(p);
            std::cout << "added point";
        }*/

        geometry_msgs::Point p;
        p.x = 0;
        p.y = 0;
        p.z = 0;

        points.points.push_back(p);
        line_strip.points.push_back(p);

        // The line list needs two points for each line
        
        p.x += 2.0;
        p.y-=0.8;
        points.points.push_back(p);
        line_strip.points.push_back(p);

        p.y += 1.0;
        p.x+=1.0;
        points.points.push_back(p);
        line_strip.points.push_back(p);

        p.y -= 0.7;
        p.x+=0.6;
        points.points.push_back(p);
        line_strip.points.push_back(p);

        p.y += 0.3;
        p.x+=2.3;
        points.points.push_back(p);
        line_strip.points.push_back(p);

        p.y -= 0.4;
        p.x+=1.3;
        points.points.push_back(p);
        line_strip.points.push_back(p);

        p.y += 0.8;
        p.x+=4.5;
        points.points.push_back(p);
        line_strip.points.push_back(p);

        


        //vis_pub.publish(points);
        //vis_pub.publish(line_strip); !!!
        //vis_pub.publish(line_list);
    }

    void odom_callback(const nav_msgs::Odometry &msg)
    {
        //points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time();

        //vis_pub.publish(line_strip); !!
        //vis_pub.publish(points);  !!
        //vis_pub.publish(line_list);

        /*points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time();
        geometry_msgs::Point p;
        p.x = msg.twist.twist.linear.x;
        p.y = msg.twist.twist.linear.y;
        p.z = msg.twist.twist.linear.z;

        points.points.push_back(p);
        line_strip.points.push_back(p);

        // The line list needs two points for each line
        line_list.points.push_back(p);
        p.z += 1.0;
        line_list.points.push_back(p);

        //vis_pub.publish(points);
        vis_pub.publish(line_strip);
        //vis_pub.publish(line_list);*/

        //std::cout << msg.pose.pose.position.z << std::endl;
        /*visualization_msgs::Marker marker;
        marker.header.frame_id = "base_link";
        marker.header.stamp = ros::Time();
        marker.ns = "my_namespace";
        marker.id = 4;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = msg.twist.twist.linear.x;
        marker.pose.position.y = msg.twist.twist.linear.y;
        marker.pose.position.z = msg.twist.twist.linear.z;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        //only if using a MESH_RESOURCE marker type:
        marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
        vis_pub.publish(marker);*/

        // publishing is done in odom callback just so it's at the same rate as the sim

        // initialize message to be published
        ackermann_msgs::AckermannDriveStamped drive_st_msg;
        ackermann_msgs::AckermannDrive drive_msg;

        /// SPEED CALCULATION:
        // set constant speed to be half of max speed
        drive_msg.speed = max_speed / 2.0;

        /// STEERING ANGLE CALCULATION
        // random number between 0 and 1
        double random = ((double)rand() / RAND_MAX);
        // good range to cause lots of turning
        double range = max_steering_angle / 2.0;
        // compute random amount to change desired angle by (between -range and range)
        double rand_ang = range * random - range / 2.0;

        // sometimes change sign so it turns more (basically add bias to continue turning in current direction)
        random = ((double)rand() / RAND_MAX);
        if ((random > .8) && (prev_angle != 0))
        {
            double sign_rand = rand_ang / std::abs(rand_ang);
            double sign_prev = prev_angle / std::abs(prev_angle);
            rand_ang *= sign_rand * sign_prev;
        }

        // set angle (add random change to previous angle)
        drive_msg.steering_angle = std::min(std::max(prev_angle + rand_ang, -max_steering_angle), max_steering_angle);

        // reset previous desired angle
        prev_angle = drive_msg.steering_angle;

        // set drive message in drive stamped message
        drive_st_msg.drive = drive_msg;

        // publish AckermannDriveStamped message to drive topic
        drive_pub.publish(drive_st_msg);
    }

}; // end of class definition

int main(int argc, char **argv)
{
    ros::init(argc, argv, "random_walker");
    RandomWalker rw;
    ros::spin();
    return 0;
}
