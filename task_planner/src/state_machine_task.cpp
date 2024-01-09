#include "ros/ros.h"
#include "TaskFunction.h"
#include <iostream>


int main(int argc, char **argv){
    ros::init(argc, argv, "state_machine_task");
    ros::NodeHandle n;
    
    ROS_INFO("--------------OBJECT DETECTION-----------------\n");


    ROS_INFO("--------------INVERSE KINEMATIC----------------\n");
    double xefPROVA[3] = {0, 0, 0};
    double phiefPROVA[3] = {1.5, 0.0, 0.0};
    std::vector<double> q = ask_inverse_kinematic(n, xefPROVA, phiefPROVA);
   

    ROS_INFO("---------------MOTION OF THE ARM ----------------\n");
    control_gazebo_arm(n,q);

    return 0;
}