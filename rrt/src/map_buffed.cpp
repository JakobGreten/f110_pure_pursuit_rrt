#include "rrt/map_buffed.h"

MAPBUFFED::~MAPBUFFED() {
    ROS_INFO("MAP_BUFFED shutting down");
}

MAPBUFFED::MAPBUFFED(ros::NodeHandle &nh) : nh_(nh) {
    nh_.getParam("rrt/map_topic", map_topic);
    nh_.getParam("rrt/map_buffed_topic", map_buffed_topic);
    nh_.getParam("rrt/buff_area", buff_area);
    map_buffed_pub = nh_.advertise<nav_msgs::OccupancyGrid>(map_buffed_topic, 1, true);

    ROS_INFO_STREAM("The map buffer started!");
    map_sub = nh_.subscribe(map_topic, 1, &MAPBUFFED::map_callback, this);
}

void MAPBUFFED::map_callback(const nav_msgs::OccupancyGrid &msg) {
    nav_msgs::OccupancyGrid oGrid;
    std::vector<int8_t> map(msg.data.size());
    map = msg.data;
    ROS_INFO_STREAM("the area is: "<< buff_area );

   // Every occupied pixel hast to be buffed.
   // If it finds one, it makes all the pixel in an area around it also occupied.
    for(int num=0; num<msg.info.height*msg.info.width; num++){
        if (msg.data[num]==100){
            for(int i = -buff_area; i<= buff_area ;i++){        
                for (int j = -buff_area; j <= buff_area; j++) {
                    int res = num+(msg.info.width*i)+j;
                    if (res < msg.info.height*msg.info.width && res>0){
                        map[res] = 100;
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
    
    map_buffed_pub.publish(oGrid);
    ROS_INFO_STREAM("The map got buffed!");
}