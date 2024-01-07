#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense> 
#include <ros/ros.h>
#include "../include/Kinematic.h"
#include "motion_planner/InverseKinematic.h"
#include <iostream>

bool inverse(motion_planner::InverseKinematic::Request &req, motion_planner::InverseKinematic::Response &res){
    double scaleFactor = 1.0;

    ROS_INFO("\n");
    ROS_INFO("REQUEST------------------------------\n");
    ROS_INFO("--REQUEST JOINT PARAMETERS ----------");
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
    ROS_INFO("\n");
    ROS_INFO("--DERIVE xe, Re and RPY of END EFFECTOR------");
    ROS_INFO("Vector xe: %s", vectorToString(xe).c_str());
    ROS_INFO("Matrix Re:%s", matrixToString(Re).c_str());
    ROS_INFO("Vector RPY: %s \n", vectorToString(Re.eulerAngles(0, 1, 2)).c_str());

    ROS_INFO("--REQUEST DESIRED END EFFECTOR ----------");
    ROS_INFO("xef : %f, %f, %f", req.xef[0], req.xef[1], req.xef[2]);
    ROS_INFO("phief : %f, %f, %f", req.phief[0], req.phief[1], req.phief[2]);

    // converto il vettore di double req.phief ed xef in vettori eigen phief e xef
    Eigen::VectorXd phief = Eigen::Map<Eigen::VectorXd>(req.phief.data(), req.phief.size());
    Eigen::VectorXd xef = Eigen::Map<Eigen::VectorXd>(req.xef.data(), req.xef.size());

    // derivo la matrice di rotazione desiderata dell'end effector da phief
    Eigen::Matrix3d Ref;
    Ref = euler2RotationMatrix(phief, "XYZ");
    ROS_INFO("Matrix Ref:%s \n", matrixToString(Ref).c_str());

    // assempblo la matrice di configurazione dell'end effector usando Ref e xef
    Eigen::Matrix4d Tt0 = Eigen::Matrix4d::Identity();
    Tt0.block<3, 3>(0, 0) = Ref;
    Tt0.block<3, 1>(0, 3) = xef;

    //prova temporanea semplice cinematica inversa
    Eigen::MatrixXd THf = cinematicaInversa(xef, Ref, scaleFactor);
    Eigen::VectorXd M = getFirstColumnWithoutNaN(THf);
    ROS_INFO("--DERIVE the first possible JOIN CONFIGURATION to reach xef------");
    ROS_INFO("Vector M: %s", vectorToString(M).c_str());
    
    std::stringstream qs;
    for(int i=0; i<6; i++){
        res.q[i] = M(i);
        qs << res.q[i] << " ";
    }
    ROS_INFO("sending back response: %s", qs.str().c_str());
    return true;
}


int main(int argc, char **argv){

    
    
    ros::init(argc, argv, "inverse_kinemtic_node");
    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("calculate_inverse_kinematics", inverse);
    ros::spin();
    
    
    
    return 0;
} 