#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <std_msgs/Float64MultiArray.h>
#include <cmath>
#include <float.h>
#include <tf/tf.h>

class PurePursuit
{
private:
    ros::NodeHandle n;
    ros::Subscriber pose_sub;
    ros::Subscriber path_sub;

    // Publish drive data
    ros::Publisher drive_pub;

    ros::Publisher vis_pub;

    double max_speed, max_steering_angle, pose_x, pose_y, wheelbase, vel;
    int sphere_marker_idx, cylinder_marker_idx, line_marker_idx;
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
        n.getParam("pure_pursuit_node/wheelbase", wheelbase);

        drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 10);
        vis_pub = n.advertise<visualization_msgs::Marker>("pure_pursuit_marker", 10);

        pose_sub = n.subscribe(pose_topic, 10, &PurePursuit::pose_callback, this);
        path_sub = n.subscribe("/path", 10, &PurePursuit::path_callback, this);
    }

    void pose_callback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg)
    {

        sphere_marker_idx = 0;
        cylinder_marker_idx = 0;
        line_marker_idx = 0;

        pose_x = pose_msg->pose.position.x;
        pose_y = pose_msg->pose.position.y;

        //target look-ahead-distance, might differ from l
        double target_l = 1.4;
        //double target_l = vel/max_speed*4+0.7;
        ackermann_msgs::AckermannDriveStamped drive_st_msg;
        ackermann_msgs::AckermannDrive drive_msg;

        double prev_node_x;
        double prev_node_y;
        double prev_diff_from_l;

        double min_diff_x;
        double min_diff_y;
        double min_diff_from_l = DBL_MAX;

        double track_node_x = NAN;
        double track_node_y = NAN;

        double a_x = NAN;
        double a_y = NAN;
        double b_x = NAN;
        double b_y = NAN;
        if (path.size() == 0 &&false)
        {
            ROS_INFO("Empty path");
        }
        for (int i = 0; (i * 2) < path.size(); i++)
        {

            //ROS_INFO_STREAM("node" << i);
            double node_x = path[path.size() - 2 * i - 2];
            double node_y = path[path.size() - 2 * i - 1];
            double distance_to_node = distance(pose_x, pose_y, node_x, node_y);
            double diff_from_l = distance_to_node - target_l;

            if (sgn(diff_from_l) != sgn(prev_diff_from_l))
            {
                //track_node_x = node_x;
                //track_node_y = node_y;
                b_x = node_x;
                b_y = node_y;
                a_x = prev_node_x;
                a_y = prev_node_y;
            }

            if (diff_from_l < min_diff_from_l)
            {
                //ROS_INFO_STREAM(diff_from_l);
                min_diff_from_l = diff_from_l;
                min_diff_x = node_x;
                min_diff_y = node_y;
            }

            prev_diff_from_l = diff_from_l;
            prev_node_x = node_x;
            prev_node_y = node_y;
        }
        if (isnanl(a_x))
        {
            track_node_x = min_diff_x;
            track_node_y = min_diff_y;
        }
        else
        {
            publishSphere(a_x, a_y);
            publishSphere(b_x, b_y);
            std::vector<double> target = trangulateTarget(a_x, a_y, b_x, b_y, pose_x, pose_y, target_l);
            track_node_x = target[0];
            track_node_y = target[1];
        }
        //actual look-ahead-distance
        double l = distance(pose_x, pose_y, track_node_x, track_node_y);
        //ROS_INFO_STREAM("l"<<l);
        publishSphere(track_node_x, track_node_y);
        publishLine(pose_x, pose_y, track_node_x, track_node_y);
        // TODO: transform goal point to vehicle frame of reference

        // TODO: calculate curvature/steering angle

        double theta = tf::getYaw(pose_msg->pose.orientation);
        double alpha = atan2((track_node_y - pose_y), (track_node_x - pose_x)) - theta;
        vel = max_speed/7;
        //ROS_INFO_STREAM("alpha" << alpha << " vel" << vel);
        if (alpha > M_PI)
        {
            alpha -= 2 * M_PI;
        }
        else if (alpha <= -M_PI)
        {
            alpha += 2 * M_PI;
        }
        //vel = max_speed / (pow(1.1, fabs(alpha)))+0.3;
        //vel = max_speed - fabs(alpha) * 2.8 + 0.8;

        //double omega = 2 * vel * sin(alpha) / l;
        double curvature = 2 * sin(alpha) / l;
        double omega = atan(curvature * wheelbase);

        //Calculate center of turning circle
        //double r = fabs(distance(pose_x, pose_y, track_node_x, track_node_y) / (2 * sin(alpha)));
        double r = fabs(1 / curvature);
        //vel = 2*r;

        double xa = 0.5 * (track_node_x - pose_x);
        double ya = 0.5 * (track_node_y - pose_y);
        double a = sqrt(xa * xa + ya * ya);
        double b = sqrt(fabs(r * r - a * a));
        double x0 = pose_x + xa;
        double y0 = pose_y + ya;
        double x_center;
        double y_center;
        if (omega > 0)
        {
            x_center = x0 - b * ya / a;
            y_center = y0 + b * xa / a;
        }
        else
        {
            x_center = x0 + b * ya / a;
            y_center = y0 - b * xa / a;
        }
        //ROS_INFO_STREAM("xcenter: " << x_center << " ycenter" << y_center << "test" << r);
        publishCylinder(x_center, y_center, r * 2);

        //ROS_INFO_STREAM("Orientation: " << theta * 180 / M_PI << " Alpha: " << alpha * 180 / M_PI << " Omega: " << omega * 180 / M_PI);

        drive_msg.speed = vel;
        drive_msg.steering_angle = omega;
        drive_st_msg.drive = drive_msg;
        drive_pub.publish(drive_st_msg);

        // TODO: publish drive message, don't forget to limit the steering angle between -0.4189 and 0.4189 radians
    }

    std::vector<double> trangulateTarget(double a_x, double a_y, double b_x, double b_y, double c_x, double c_y, double l)
    {
        double a = distance(c_x, c_y, b_x, b_y);
        double b = distance(c_x, c_y, a_x, a_y);
        double c = distance(b_x, b_y, a_x, a_y);

        double cosalpha = (b * b + c * c - a * a) / 2 * b * c;
        double c1 = b * cosalpha + sqrt(b * b * cosalpha * cosalpha - b * b + l * l);
        double c2 = b * cosalpha - sqrt(b * b * cosalpha * cosalpha - b * b + l * l);

        double c3;
        if ((c1 < c && c1 > c2) || c2 > c)
        {
            c3 = c1;
            //ROS_INFO_STREAM("c1");
        }
        else
        {
            c3 = c2;
            //ROS_INFO_STREAM("c2");
        }

        double t_x = a_x + (b_x - a_x) / c * c3;
        double t_y = a_y + (b_y - a_y) / c * c3;
        //publishSphere(t_x, t_y);
        std::vector<double> target;
        target.push_back(t_x);
        target.push_back(t_y);
        return target;
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
        double size = 0.2;
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
    void publishLine(double ax, double ay, double bx, double by)
    {
        visualization_msgs::Marker marker, points;
        marker.header.frame_id = "/map";
        marker.header.stamp = ros::Time::now();

        marker.ns = "lines";
        marker.id = line_marker_idx;
        line_marker_idx++;

        marker.type = visualization_msgs::Marker::LINE_LIST;
        points.type = visualization_msgs::Marker::POINTS;

        marker.action = visualization_msgs::Marker::ADD;

        geometry_msgs::Point a;
        a.x = ax;
        a.y = ay;
        a.z = 0;
        points.points.push_back(a);
        marker.points.push_back(a);

        geometry_msgs::Point b;
        b.x = bx;
        b.y = by;
        b.z = 0;
        points.points.push_back(b);
        marker.points.push_back(b);

        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        double size = 0.08;
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
    void publishCylinder(double x, double y, double diameter)
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

        marker.color.r = 0.5f;
        marker.color.g = 0.5f;
        marker.color.b = 0.5f;
        marker.color.a = 0.5 / pow(diameter, 0.1);
        //ROS_INFO_STREAM(diameter);
        marker.lifetime = ros::Duration();
        vis_pub.publish(marker);
    }
    template <typename T>
    int sgn(T val)
    {
        return (T(0) < val) - (val < T(0));
    }
};
int main(int argc, char **argv)
{
    ros::init(argc, argv, "pure_pursuit_node");
    PurePursuit pp;
    ros::spin();
    return 0;
}