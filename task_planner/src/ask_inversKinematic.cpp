#include "ros/ros.h"
#include "motion_planner/InverseKinematic.h"
#include <cstdlib>

int main(int argc, char **argv){

    ros::init(argc, argv, "ask_invers_kinematic_node");
    if (argc != 1) {
        ROS_INFO("usage: ask_invers_kinematic_node X Y Z");
        return 1;
    }

    ros::NodeHandle n;
    ros::ServiceClient service_client = n.serviceClient<motion_planner::InverseKinematic>("calculate_inverse_kinematics");
    motion_planner::InverseKinematic srv;
    srv.request.x = 1.3;
    srv.request.y = 1.2;
    srv.request.z = 2;

    if (service_client.call(srv)){
        ROS_INFO("q: %f", srv.response.q);
    } else {
        ROS_ERROR("Failed to call service add_two_ints");
        return 1;
    }
    
    return 0;
}