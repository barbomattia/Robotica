#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense> 
#include <ros/ros.h>
#include "../include/Kinematic.h"
#include "motion_planner/InverseKinematic.h"
#include <iostream>

bool inverse(motion_planner::InverseKinematic::Request &req, motion_planner::InverseKinematic::Response &res){
    double scaleFactor = 10.0;
    double Tf = 10.0; 
    double DeltaT = 0.1;
    Eigen::VectorXd T;
    T = Eigen::VectorXd::LinSpaced(static_cast<int>((Tf / DeltaT) + 1), 0, Tf);

    ROS_INFO("\n");
    ROS_INFO("REQUEST------------------------------\n");
    ROS_INFO("--REQUEST JOINT PARAMETERS ----------");
    for (int i = 0; i < req.jointstate.size(); i++) {
        ROS_INFO("joint %d: %f", i, req.jointstate[i]);
    }
    
    // converto il vettore di double req.jointstate in un vettore eigen joinstate
    Eigen::VectorXd jointstate = Eigen::Map<Eigen::VectorXd>(req.jointstate.data(), req.jointstate.size());

    // calcolo la configurazione attuale dell'end effector sapendo lo stato dei joint
    auto result = CinematicaDiretta(jointstate, scaleFactor);   
    Eigen::VectorXd xe = result.pe;     // posizione end effector 
    Eigen::Matrix3d Re = result.Re;     // rotazione end effector 
    Eigen::Quaterniond q0(Re);          // quaternione configurazione iniziale 

    // Stampa del vettore xe e della matrice Re
    std::cout << std::endl;
    ROS_INFO("--DERIVE INITIAL INFORMATION xe, Re, RPY, q0 of END EFFECTOR------\n");
    std::cout << "Vector xe: " << vectorToString(xe).c_str() << std::endl;
    std::cout << "Matrix Re: " << matrix3dToString(Re).c_str() << std::endl;
    std::cout << "Vector RPY: " << vectorToString(Re.eulerAngles(0, 1, 2)).c_str() << std::endl;
    std::cout << "Quaternion q0: " << quaternioToString(q0).c_str() << std::endl << std::endl; 

    ROS_INFO("--REQUEST DESIRED END EFFECTOR ----------\n");
    for(int i=0; i<3; i++){
        req.xef[i] = req.xef[i] * scaleFactor;
    }
    std::cout << "Vector Location xef: " << req.xef[0] << ", " << req.xef[1] << ", " << req.xef[2] << std::endl;
    std::cout << "Vector Euler phief: " << req.phief[0] << ", " << req.phief[1] << ", " << req.phief[2] << std::endl;

   
    // converto il vettore di double req.phief ed xef in vettori eigen phief e xef
    Eigen::VectorXd phief = Eigen::Map<Eigen::VectorXd>(req.phief.data(), req.phief.size());
    Eigen::VectorXd xef = Eigen::Map<Eigen::VectorXd>(req.xef.data(), req.xef.size());

    // derivo la matrice di rotazione desiderata dell'end effector da phief
    Eigen::Matrix3d Ref;
    Ref = euler2RotationMatrix(phief, "XYZ");
    std::cout << "Matrix Ref: " << matrix3dToString(Ref).c_str() << std::endl;

    // assempblo la matrice di configurazione dell'end effector usando Ref e xef
    Eigen::Matrix4d Tt0 = Eigen::Matrix4d::Identity();
    Tt0.block<3, 3>(0, 0) = Ref;
    Tt0.block<3, 1>(0, 3) = xef;
    Eigen::Quaterniond qf(Tt0.block<3, 3>(0, 0));
    std::cout << "Quaternion qf: " << quaternioToString(qf).c_str() << std::endl;

    /*prova temporanea semplice cinematica inversa
    Eigen::MatrixXd THf = cinematicaInversa(xef, Ref, scaleFactor);
    Eigen::VectorXd M = getFirstColumnWithoutNaN(THf);
    ROS_INFO("--DERIVE the first possible JOIN CONFIGURATION to reach xef------");
    ROS_INFO("%s", vectorToString(M).c_str());
    */

    Eigen::Matrix3d Kp = 3 * Eigen::Matrix3d::Identity();
    Eigen::Matrix3d Kq = -2.65 * Eigen::Matrix3d::Identity();

    Eigen::MatrixXd Th = invDiffKinematicControlSimCompleteQuaternion(jointstate, Kp, Kq, T, 0.0, Tf, DeltaT, scaleFactor, Tf, xe, xef, q0, qf);
    ROS_INFO("--DERIVED q ------");
    std::cout << "Dimensioni di Th: " << Th.rows() << " " << Th.cols() << std::endl << matrixToString(Th).c_str() << std::endl;

    //copio la matrice th nella risposta
    for (int i = 0; i < Th.rows(); i++) { 
        for (int j = 0; j < Th.cols(); j++) {
            //res.array_q.push_back(Th(i,j));
            if (i >= 0 && i < Th.rows() && j >= 0 && j < Th.cols()) {
                res.array_q.push_back(Th(i, j));
            } else {
                ROS_ERROR("Indici fuori dai limiti: i=%d, j=%d", i, j);
            }
        }
    }

    for(int i=0; i < res.array_q.size(); i++){
        std::cout << res.array_q[i] << " ";
    }
    std::cout<<"\n";

    ROS_INFO("-- END REQUEST ------");

    
    return true;
}

int main(int argc, char **argv){

    ros::init(argc, argv, "inverse_kinemtic_node");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("calculate_inverse_kinematics", inverse);

    ros::spin();
     
    return 0;
} 