// This file is part of the 2019 skeleton code(https://github.com/mlab-upenn/f110-fall2019-skeletons) 
// While the skeleton code provided an initial structure, the majority of the code 
// was written by Steffen Fleischmann, Jakob Greten, Kilian Poeschel and Joshua Bahn.
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
//This class implements a pure pursuit controller for the f110 platform
class PurePursuit
{
private:
    ros::NodeHandle n;
    //Subscribers
    ros::Subscriber pose_sub;
    ros::Subscriber path_sub;

    // Publishers
    ros::Publisher drive_pub;
    ros::Publisher vis_pub;

    double max_speed, max_steering_angle, pose_x, pose_y, wheelbase, vel;
    double target_l, vel_mult_factor, vel_add_factor, max_mult_speed;
    int sphere_marker_idx, cylinder_marker_idx, line_marker_idx;
    bool goal_reached = false;
    //rrt path
    std::vector<double> path;

public:
    //Constructor of pure pursuit. Initiates publishers and subscribers
    PurePursuit()
    {
        n = ros::NodeHandle();
        std::string drive_topic, pose_topic;

        //get parameters
        n.getParam("pure_pursuit_node/pp_drive_topic", drive_topic);
        //n.getParam("/pose_topic", pose_topic);
        pose_topic = "/gt_pose"; //other pose topic not working
        //pose_topic = "/vrpn_client_node/f1tenth/pose";
        n.getParam("pure_pursuit_node/max_speed", max_speed);
        n.getParam("pure_pursuit_node/max_steering_angle", max_steering_angle);
        n.getParam("pure_pursuit_node/wheelbase", wheelbase);
        n.getParam("pure_pursuit_node/target_l", target_l);
        n.getParam("pure_pursuit_node/vel_mult_factor", vel_mult_factor);
        n.getParam("pure_pursuit_node/vel_add_factor", vel_add_factor);
        n.getParam("pure_pursuit_node/max_mult_speed", max_mult_speed);


        //differentiate between simualted and real launch
        bool real = false;
        std::string real_pose_topic = "";
        n.getParam("/real", real);
        n.getParam("/real_pose_topic", real_pose_topic);
        if(real){
            pose_topic = real_pose_topic;
            ROS_INFO_STREAM("Real Pure Pursuit launch");
        }

        //initiate publishers
        drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 10);
        vis_pub = n.advertise<visualization_msgs::Marker>("pure_pursuit_marker", 10);

        //initiate subscribers
        pose_sub = n.subscribe(pose_topic, 10, &PurePursuit::pose_callback, this);
        path_sub = n.subscribe("/path", 10, &PurePursuit::path_callback, this);
    }

    // This callback receives the current position. 
    //Initiates the calculation of new steering angle and velocity 
    void pose_callback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg)
    {
        //publish origin of coordinate frame
        //publishSphere(0.0,0.0);

        //indexes of markers for rviz
        sphere_marker_idx = 0;
        cylinder_marker_idx = 0;
        line_marker_idx = 0;
        
        //update pose
        pose_x = pose_msg->pose.position.x;
        pose_y = pose_msg->pose.position.y;

        //target look-ahead-distance, might differ from l
        //double target_l = 1.0;
        //double target_l = vel/max_speed*1.4+0.9;


        ackermann_msgs::AckermannDriveStamped drive_st_msg;
        ackermann_msgs::AckermannDrive drive_msg;

        std::vector<double> track_node= getLookAheadPoint(target_l);

        double track_node_x=track_node[0];
        double track_node_y=track_node[1];

        //actual look-ahead-distance, might differ from target_l
        double l = distance(pose_x, pose_y, track_node_x, track_node_y);

        //publish track node for debugging
        publishSphere(track_node_x, track_node_y);
        //publish look ahead distance
        publishLine(pose_x, pose_y, track_node_x, track_node_y);

        //current angle of car
        double theta = tf::getYaw(pose_msg->pose.orientation);

        //angle between orientation of car and target node
        double alpha = atan2((track_node_y - pose_y), (track_node_x - pose_x)) - theta;

        //limit alpha in interval [-pi,pi]
        if (alpha > M_PI)
        {
            alpha -= 2 * M_PI;
        }
        else if (alpha <= -M_PI)
        {
            alpha += 2 * M_PI;
        }

        updateVelocity(alpha);
        

        double curvature = 2 * sin(alpha) / l;
        double omega = atan(curvature * wheelbase);

        //Calculate radius and center of turning circle
        double r = fabs(1 / curvature);

        drawTurningCircle(r,omega, track_node_x, track_node_y);

        //limit steering angle in correct interval
        if(omega>0.4189){
            omega = 0.4189;
        }else if (omega<-0.4189){
            omega = -0.4189;
        }
        
        

        //ROS_INFO_STREAM("Orientation: " << theta * 180 / M_PI << " Alpha: " << alpha * 180 / M_PI << " Omega: " << omega * 180 / M_PI);

        //publish drive message with velocity and steering angle
        drive_msg.speed = vel;
        drive_msg.steering_angle = omega;
        drive_st_msg.drive = drive_msg;
        drive_pub.publish(drive_st_msg);
    }

    //  This method draws the calculated turning circle to visualize the controller behaviour.
    //  Args:
    //   r (double): radius of the circle
    //   omega(double): steering angle of the car
    //   track_node_x(double): look ahead point x
    //   track_node_y(double): look ahead point y
    void drawTurningCircle(double r, double omega, double track_node_x, double track_node_y){
        //Calculate cooridnates of the center of circle with radius r, which lines up
        //with both the pose of the car and the target point. This is only used for visualization.
        double xa = 0.5 * (track_node_x - pose_x);
        double ya = 0.5 * (track_node_y - pose_y);
        double a = sqrt(xa * xa + ya * ya);
        double b = sqrt(fabs(r * r - a * a));
        double x0 = pose_x + xa;
        double y0 = pose_y + ya;
        double x_center;
        double y_center;

        //Turning circle can be on both sides of the trajectory.
        //Picking the correct one based on steering angle
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
        //publish turning circle
        publishCylinder(x_center, y_center, r * 2);
    }

    // This method calculates the target velocity of the car
    // Args:
    //   alpha (double): angle between car orientation and look ahead point
    // Returns:
    //   vel (std::vector<double> ) target velocity
    void updateVelocity(double alpha){
        //stop the car if the goal is reached or if no path was found
        if (goal_reached || path.size()==0)
        {
            vel = 0;
        }
        //Calculate new velocity
        else
        {   
            //parameters for velocity. Determined through trial and error
            //double vel_mult_factor = 6.0;
            //double vel_add_factor = 0.9;

            //velocity decreases quadratically to alpha
            double mult_part = max_mult_speed - fabs(alpha) * vel_mult_factor;

            //make sure velocity is positive
            if (mult_part > 0)
            {
                vel = mult_part + vel_add_factor;
            }
            else
            {
                vel = vel_add_factor;
            }

        }
    }
    
    // This method calculates the look ahead point on the path which has 
    // a distance of target_l from the car
    // Args:
    //   target_l (double): distance between target and car
    // Returns:
    //   track_node (std::vector<double> ) look ahead point
    std::vector<double> getLookAheadPoint(double target_l){

        double prev_node_x;
        double prev_node_y;
        //difference from l from prev node
        double prev_diff_from_l;

        double min_diff_x;
        double min_diff_y;

        //current minimal diff from l from all nodes
        double min_diff_from_l = DBL_MAX;
        
        //node currently being tracked
        double track_node_x = NAN;
        double track_node_y = NAN;

        //look ahead point is located between a and b
        double a_x = NAN;
        double a_y = NAN;
        double b_x = NAN;
        double b_y = NAN;

        //check if goal is already reached
        if(path.size()>0 && distance(pose_x,pose_y,path[0],path[1])<0.75){
            goal_reached = true;
        }else{
            goal_reached = false;
        }

        //iterate through path and look for the point with a distance of l towards current pose
        for (int i = 0; (i * 2) < path.size(); i++)
        {

            double node_x = path[path.size() - 2 * i - 2];
            double node_y = path[path.size() - 2 * i - 1];

            
            double distance_to_node = distance(pose_x, pose_y, node_x, node_y);

            //difference form target distance
            double diff_from_l = distance_to_node - target_l;
            
            //if the sign of diff_from_l changes, the target point is located between this
            //and the previous node
            //ROS_INFO_STREAM(diff_from_l<<" prev"<<prev_diff_from_l);
            if (signnum(diff_from_l) != signnum(prev_diff_from_l)||diff_from_l<0.1)
            {
                //double next_diff=distance(pose_x, pose_y, path[path.size() - 2 * (i+1) - 2], path[path.size() - 2 * (i+1) - 1]);
                //ROS_INFO_STREAM("i "<< i<<" diff "<<diff_from_l<<" next diff "<<next_diff<<" prev "<<prev_diff_from_l);
                b_x = node_x;
                b_y = node_y;
                a_x = prev_node_x;
                a_y = prev_node_y;
                
            }

            //check if this is the smallest diff_from_l
            if (diff_from_l < min_diff_from_l)
            {
                min_diff_from_l = diff_from_l;
                min_diff_x = node_x;
                min_diff_y = node_y;
            }

            prev_diff_from_l = diff_from_l;
            prev_node_x = node_x;
            prev_node_y = node_y;
        }

        //if no point with exactly l diff was found, chose the node closest to target distance
        if (isnanl(a_x))
        {
            track_node_x = min_diff_x;
            track_node_y = min_diff_y;
        }
        //otherwise calculate exact coodinates of target point betwen a and b.
        else
        {
            //publishSphere(a_x, a_y);
            //publishSphere(b_x, b_y);
            std::vector<double> target = triangulateTarget(a_x, a_y, b_x, b_y, pose_x, pose_y, target_l);
            track_node_x = target[0];
            track_node_y = target[1];
        }

        std::vector<double> track_node;
        track_node.push_back(track_node_x);
        track_node.push_back(track_node_y);
        return track_node;



    }

    // This method triangulates the target point between a and b, 
    // which is also at a distance of l to point c
    // Args:
    //   a_x (double): x coordinate of point a
    //   a_y (double): y coordinate of point a
    //   b_x (double): x coordinate of point b
    //   b_y (double): y coordinate of point b
    //   c_x (double): x coordinate of point c
    //   c_y (double): y coordinate of point c
    //   l (double): target distance l
    // Returns:
    //   target (std::vector<double> ) triangulated target point

    std::vector<double> triangulateTarget(double a_x, double a_y, double b_x, double b_y, double c_x, double c_y, double l)
    {
        //lengths of triangle
        double a = distance(c_x, c_y, b_x, b_y);
        double b = distance(c_x, c_y, a_x, a_y);
        double c = distance(b_x, b_y, a_x, a_y);

        //cos of angle at point a
        double cosalpha = (b * b + c * c - a * a) / (2 * b * c);

        //distance between point A and target point. Up to two solutions 
        double c1 = b * cosalpha + sqrt(b * b * cosalpha * cosalpha - b * b + l * l);
        double c2 = b * cosalpha - sqrt(b * b * cosalpha * cosalpha - b * b + l * l);

        //chosen distance between A and target point
        double c3;
        
        //chose max(c1,c2) but verify that this is between 0 and c 
        //if ((c1 < c && c1 > c2 && c1 > 0) || (c2 > c && c1 < c && c1 > 0))
        if(c1<c && c1>0 && (c1>c2||c2<0||c2>c))
        {
            c3 = c1;
        }
        else if (c2 < c && c2 > 0)
        {
            c3 = c2;
        }
        //if both c1 and c2 are not in range, chose c
        else
        {
            c3 = c;

        }

        //calculate coordinates of target point using c3
        double t_x = a_x + (b_x - a_x) / c * c3;
        double t_y = a_y + (b_y - a_y) / c * c3;


        std::vector<double> target;
        target.push_back(t_x);
        target.push_back(t_y);
        return target;
    }

    //This callback receives the path from the rrt node
    // Args:
    //   msg (std_msgs::Float64MultiArray): the path calculated by rrt
    void path_callback(const std_msgs::Float64MultiArray::ConstPtr &msg)
    {
        //update path
        path = msg->data;
    }



    //Calculates the distance between two points
    // Args:
    //   a_x (double): x coordinate of point a
    //   a_y (double): y coordinate of point a
    //   b_x (double): x coordinate of point b
    //   b_y (double): y coordinate of point b
    double distance(double ax, double ay, double bx, double by)
    {
        double xdif = ax - bx;
        double ydif = ay - by;
        double distance = sqrt(xdif * xdif + ydif * ydif);
        return distance;
    }

    // Publishes a sphere for debugging
    // Args:
    //   x (double): x coordinate of sphere
    //   y (double): y coordinate of sphere
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

    // Publishes a line for debugging between point a and b
    // Args:
    //   a_x (double): x coordinate of point a
    //   a_y (double): y coordinate of point a
    //   b_x (double): x coordinate of point b
    //   b_y (double): y coordinate of point b
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

    // Publishes a cylinder for debugging with given diameter
    // Args:
    //   x (double): x coordinate of sphere
    //   y (double): y coordinate of sphere
    //   diameter (double): diameter of cylinder
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

        marker.lifetime = ros::Duration();
        vis_pub.publish(marker);
    }

    // This method returns the sign of T
    // Args:
    //   T(typename): any variable
    // Returns:
    //  sign (int) sign of T
    /*template <typename T>
    int sgn(T val)
    {
        return (T(0) < val) - (val < T(0));
    }*/
    double signnum(double x){
        if(x>=0.0)return 1;
        return -1;
    }
};
//Main method for pure pursuit. Starts ros node
int main(int argc, char **argv)
{
    ros::init(argc, argv, "pure_pursuit_node");
    PurePursuit pp;
    ros::spin();
    return 0;
}
