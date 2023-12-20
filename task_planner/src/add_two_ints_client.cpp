#include "ros/ros.h"
#include "motion_planner/AddTwoInts.h"
#include <cstdlib>

int main(int argc, char **argv){
    ros::init(argc, argv, "add_two_ints_client");
    if (argc != 3) {
        ROS_INFO("usage: add_two_ints_client X Y");
        return 1;
    }

    ros::NodeHandle n;
    ros::ServiceClient service_client = n.serviceClient<motion_planner::AddTwoInts>("add_two_ints");
    motion_planner::AddTwoInts srv;
    srv.request.a = atoll(argv[1]);
    srv.request.b = atoll(argv[2]);

    if (service_client.call(srv)){
        ROS_INFO("Sum: %ld", (long int)srv.response.sum);
    } else {
        ROS_ERROR("Failed to call service add_two_ints");
        return 1;
    }
    
    return 0;
}
