#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense> 
#include <ros/ros.h>
#include "../include/Kinematic.h"
#include "motion_planner/InverseKinematic.h"
#include <iostream>


bool inverse(motion_planner::InverseKinematic::Request &req, motion_planner::InverseKinematic::Response &res){
    res.q = req.x + req.y + req.z;
    ROS_INFO("request: x=%f, y=%f, z=%f", req.x, req.y, req.z);
    ROS_INFO("sending back response: [%f]", res.q);
    return true;
}


int main(int argc, char **argv){

    
    
    ros::init(argc, argv, "inverse_kinemtic_node");
    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("calculate_inverse_kinematics", inverse);
    ros::spin();

    
    
    return 0;
} 