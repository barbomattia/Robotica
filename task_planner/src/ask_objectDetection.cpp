#include "ros/ros.h"
#include "vision_planner/objectDetection.h"
#include <cstdlib>

int main(int argc, char **argv){

    ros::init(argc, argv, "ask_object_detection_node");
    if (argc != 1) {
        ROS_INFO("usage: ask_object_detection_node ");
        return 1;
    }

    ros::NodeHandle n;
    ros::ServiceClient service_client = n.serviceClient<vision_planner::objectDetection>("object_detection");
    vision_planner::objectDetection srv;
    srv.request.img = 5;

    if (service_client.call(srv)){
        ROS_INFO("x: %f, y:%f, z:%f", srv.response.x, srv.response.y, srv.response.z );
    } else {
        ROS_ERROR("Failed to call service add_two_ints");
        return 1;
    }
    
    return 0;
}