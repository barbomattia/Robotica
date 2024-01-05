#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense> 
#include <ros/ros.h>
#include "../include/Kinematic.h"
#include "motion_planner/InverseKinematic.h"
#include <iostream>

bool inverse(motion_planner::InverseKinematic::Request &req, motion_planner::InverseKinematic::Response &res){
    res.q = 0;
    double scaleFactor = 1.0;

    ROS_INFO("request:");
    for (int i = 0; i < req.jointstate.size(); i++) {
        ROS_INFO("joint %d: %f", i, req.jointstate[i]);
    }


    // converto il vettore di double req.jointstate in un vettore eigen joinstate
    Eigen::VectorXd jointstate = Eigen::Map<Eigen::VectorXd>(req.jointstate.data(), req.jointstate.size());

    // calcolo la configurazione dell'end effector sapendo lo stato dei joint
    auto result = CinematicaDiretta(jointstate, scaleFactor);   
    Eigen::VectorXd xe = result.pe;     // posizione end effector 
    Eigen::Matrix3d Re = result.Re;     // rotazione end effector 

    // Stampa del vettore xe e della matrice Re
    ROS_INFO("Vector xe: %s", vectorToString(xe).c_str());
    ROS_INFO("Matrix Re:\n%s", matrixToString(Re).c_str());
    ROS_INFO("Vector RPY: %s", vectorToString(Re.eulerAngles(0, 1, 2)).c_str());


    


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