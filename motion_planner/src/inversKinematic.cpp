#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense> 
#include <ros/ros.h>
#include "../include/Kinematic.h"
#include "motion_planner/InverseKinematic.h"
#include <iostream>
#include <fstream>



bool inverse(motion_planner::InverseKinematic::Request &req, motion_planner::InverseKinematic::Response &res){
    double scaleFactor = 10.0;
    double Tf = 4.0; 
    double DeltaT = 0.04;
    Eigen::Matrix3d Kp = 3 * Eigen::Matrix3d::Identity();
    Eigen::Matrix3d Kq = -3 * Eigen::Matrix3d::Identity();

    Eigen::VectorXd T;
    T = Eigen::VectorXd::LinSpaced(static_cast<int>((Tf / DeltaT) + 1), 0, Tf);

    std::string outputPath = "ros_ws/src/Robotica/motion_planner/output_log/";
    if(req.first) deleteTxtFiles(outputPath);

    std::string nomeFile = req.title + ".txt";          
    std::ofstream outputFile((outputPath + nomeFile));       // Apertura del file in modalità scrittura

    // Controllo se il file è stato aperto correttamente
    if (!outputFile.is_open()) {
        std::cerr << "Impossibile aprire il file." << std::endl;
        return 1;
    }

    ROS_INFO("\n");
    if(req.grasp){ ROS_INFO("REQUEST GRASP ----------------------------------------\n"); }
    else{          ROS_INFO("REQUEST ----------------------------------------------\n"); }
   
    ROS_INFO("REQUEST JOINT PARAMETERS ");
    for (int i = 0; i < req.jointstate.size(); i++) {
        std::cout << "joint " << i << ": " << req.jointstate[i] << " ";
    }
    
    // converto il vettore di double req.jointstate in un vettore eigen joinstate
    Eigen::VectorXd jointstate = Eigen::Map<Eigen::VectorXd>(req.jointstate.data(), req.jointstate.size());

    // calcolo la configurazione attuale dell'end effector sapendo lo stato dei joint
    auto result = CinematicaDiretta(jointstate, scaleFactor);   
    Eigen::VectorXd xe = result.pe;     // posizione end effector 
    Eigen::Matrix3d Re = result.Re;     // rotazione end effector 
    Eigen::VectorXd phie = Re.eulerAngles(0, 1, 2);
    Eigen::Quaterniond q0(Re);          // quaternione configurazione iniziale 

    // Stampa del vettore xe e della matrice Re
    std::cout << std::endl;
    ROS_INFO("\nDERIVE INITIAL INFORMATION xe, Re, RPY, q0 of END EFFECTOR\n");
    std::cout << "Vector xe: " << vectorToString(xe).c_str() << std::endl;
    std::cout << "Matrix Re: " << matrix3dToString(Re).c_str() << std::endl;
    std::cout << "Vector phie: " << vectorToString(phie).c_str() << std::endl;
    std::cout << "Quaternion q0: " << quaternioToString(q0).c_str() << std::endl << std::endl; 

    ROS_INFO("REQUEST DESIRED END EFFECTOR");
    for(int i=0; i<3; i++){
        req.xef[i] = req.xef[i] * scaleFactor;
    }
    std::cout << "Vector Location xef: " << req.xef[0] << ", " << req.xef[1] << ", " << req.xef[2] << std::endl;
    std::cout << "Vector Euler phief: " << req.phief[0] << ", " << req.phief[1] << ", " << req.phief[2] << std::endl;

    
    // converto il vettore di double req.phief ed xef in vettori eigen phief e xef
    Eigen::VectorXd phief = Eigen::Map<Eigen::VectorXd>(req.phief.data(), req.phief.size());
    phief[2] = phief[2] - phie[2];
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
    std::cout << "Quaternion qf: " << quaternioToString(qf).c_str() << std::endl << std::endl;

    /*prova temporanea semplice cinematica inversa
    Eigen::MatrixXd THf = cinematicaInversa(xef, Ref, scaleFactor);
    Eigen::VectorXd M = getFirstColumnWithoutNaN(THf);
    ROS_INFO("--DERIVE the first possible JOIN CONFIGURATION to reach xef------");
    ROS_INFO("%s", vectorToString(M).c_str());
    */

    Eigen::MatrixXd Th = invDiffKinematicControlSimCompleteQuaternion(jointstate, Kp, Kq, T, 0.0, Tf, DeltaT, scaleFactor, Tf, xe, xef, q0, qf, false, outputFile);
    ROS_INFO("DERIVED q");
    std::cout << "Dimensioni di Th: " << Th.rows() << " " << Th.cols() << std::endl  << std::endl;
       
    // flag per il controllo collisioni e singolarità
    bool check = false;
    check = checkCollisionSingularity(Th, scaleFactor, req.grasp , outputFile);

    //se non ci sono collisioni ritorno la matrice Th altrimenti calcolo una traiettoria alternativa
    if(!check){

        std::cout << "Assenza di collisioni o singolarità " << std::endl << std::endl;

        //copio la matrice th nella risposta
        for (int i = 0; i < Th.rows(); i++) { 
            for (int j = 0; j < Th.cols(); j++) {
                res.array_q.push_back(Th(i, j));
            }
        }

    } else if(!req.grasp) {

        std::cout << std::endl << "Presenza di collisioni o singolarità, 2 step strategi " << std::endl;
        Th = alternativeTrajectory(jointstate, Kp, Kq, T, 0.0, Tf, DeltaT, scaleFactor, Tf, xe, xef, q0, qf, outputFile);
    
        //copio la matrice th nella risposta
        for (int i = 0; i < Th.rows(); i++) { 
            for (int j = 0; j < Th.cols(); j++) {
                    res.array_q.push_back(Th(i, j));
            }
        }

    } else {

        std::cout << std::endl << "Presenza di collisioni o singolarità, impossibile il grasping " << std::endl;
        Th.resize(1, 1);
        Th << 0.0;
    }

    outputFile << std::endl << "MATRICE di q" << std::endl << Th << std::endl << std::endl;
    
    /*
    for(int i=0; i < res.array_q.size(); i++){
        std::cout << res.array_q[i] << " ";
    }
    std::cout<<"\n";
    */

    ROS_INFO("END REQUEST ---------------------------------------------------------------------\n\n");

    
    return true;
}


int main(int argc, char **argv){

    ros::init(argc, argv, "inverse_kinemtic_node");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("calculate_inverse_kinematics", inverse);

    ros::spin();
     
    return 0;
} 