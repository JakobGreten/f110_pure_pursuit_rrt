#include "rrt/map_buffed.h"

MAPBUFFED::~MAPBUFFED() {
    ROS_INFO("MAP_BUFFED shutting down");
}

MAPBUFFED::MAPBUFFED(ros::NodeHandle &nh) : nh_(nh) {
    nh_.getParam("rrt/map_topic", map_topic);
    nh_.getParam("rrt/map_buffed_topic", map_buffed_topic);
    nh_.getParam("rrt/buff_area", buff_area); 
    //parameterized buff_area returns not the value in rrt_params.yaml.
    //I don't know why, yet.
    //buff_area = 2;
    map_buffed_pub = nh_.advertise<nav_msgs::OccupancyGrid>(map_buffed_topic, 1, true);

    ROS_INFO_STREAM("The map buffer started!");
    map_sub = nh_.subscribe(map_topic, 1, &MAPBUFFED::map_callback, this);
}

void MAPBUFFED::map_callback(const nav_msgs::OccupancyGrid &msg) {
    nav_msgs::OccupancyGrid oGrid;
    std::vector<int8_t> map(msg.data.size());
    map = msg.data;
    ROS_INFO_STREAM("the area is: "<< buff_area );

   //every pixel hast to be buffed
    for(int num=0; num<msg.info.height*msg.info.width; num++){
        if (msg.data[num]==100){
            //ROS_INFO_STREAM("pixel: "<< num << " is occupied. ");
            for(int i = -buff_area; i<= buff_area ;i++){        
                for (int j = -buff_area; j <= buff_area; j++) {
                    int res = num+(msg.info.width*i)+j;
                    //ROS_INFO_STREAM("pixel: "<< res << " is now also occupied.");
                    if (res < msg.info.height*msg.info.width && res>0){
                        map[res] = 100;
                        //ROS_INFO_STREAM("pixel: "<< res << " is now also occupied.");
                    }                    
                }
            }
        }                       
    }

    //building msg for map_buffed_pub
    oGrid.header.frame_id = "map";
    oGrid.header.stamp = ros::Time::now();

    oGrid.data = map;

    oGrid.info.height = msg.info.height;
    oGrid.info.map_load_time = ros::Time::now();
    oGrid.info.origin = msg.info.origin;
    oGrid.info.resolution = msg.info.resolution;
    oGrid.info.width = msg.info.width;
    
    //TODO: the maps origin is not in the center of the map
    map_buffed_pub.publish(oGrid);
    ROS_INFO_STREAM("The map got buffed!");
}

/*void MAPBUFFED::map_callback(const nav_msgs::OccupancyGrid &msg) {
    nav_msgs::OccupancyGrid oGrid;
    
    oGrid.header.frame_id = "map_buffed";
    oGrid.header.stamp = ros::Time::now();

    oGrid.data = msg.data;

    oGrid.info.height = msg.info.height;
    oGrid.info.map_load_time = ros::Time::now();
    oGrid.info.origin = msg.info.origin;
    oGrid.info.resolution = msg.info.resolution;
    oGrid.info.width = msg.info.width;

    ROS_INFO_STREAM("The map got buffed!");
    map_buffed_pub.publish(oGrid);
    ROS_INFO_STREAM(oGrid.header.frame_id);
    ROS_INFO_STREAM(oGrid.header.seq);
    ROS_INFO_STREAM(oGrid.header.stamp);
    ROS_INFO_STREAM(msg.header.frame_id);
    ROS_INFO_STREAM(msg.header.seq);
    ROS_INFO_STREAM(msg.header.stamp);
}*/
