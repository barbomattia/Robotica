#include "ros/ros.h"
#include "TaskFunction.h"
#include <iostream>


int main(int argc, char **argv){
    ros::init(argc, argv, "state_machine_task");
    ros::NodeHandle n;

    bool last_one_bloc = false;

    ROS_INFO("///////////////////////////////// NUOVO CICLO //////////////////////////////////////////\n");

    std::cout << " ----------------------------- OBJECT DETECTION --------------------------------- \n";
    std::vector<Block> blocchi = ask_object_detection(n);                 //ottengo tutti i blocchi trovati
    Block blocco;                                             //dichiaro il blocco che voglio ordinare    
    blocco.x[0] = 0.9; blocco.x[1] = 0.65; blocco.x[2] = 1.1;
    blocco.phi[0] = 0; blocco.phi[1] = 0; blocco.phi[2] = 0;
    blocco.name = "prova";
                                
    std::cout << "\n ----------------------------- ASK MOTION PLAN -------------------------------- \n";
    Eigen::MatrixXd q = ask_inverse_kinematic(n, blocco.x, blocco.phi);


    std::cout << "\n ----------------------------- REACHING THE BLOCK ----------------------------- \n";
    /*
    for(int i = 0; i < q.rows(); i++){

        Eigen::MatrixXd qi = q.row(i);
        // std::cout<<qi<<"\n";
        std::vector<double> qi_vector(qi.data(), qi.data() + qi.size());
        
        /*
        for (const auto& elem : qi_vector) {
            std::cout << elem << " ";
        }
        std::cout << std::endl;
        

        control_gazebo_arm(n,qi_vector);
        std::cout <<"\n";

    }
    */
    control_gazebo_arm_2(n,q);


    std::cout << "\n ----------------------------- GRASP THE BLOCK ------------------------------- \n";


    std::cout << "\n ----------------------------- PUT THE BLOCK IN ORDER ------------------------ \n";
    std::vector<double> end_block_position = define_end_position(blocco.name);
    if (isZero(end_block_position)) {
        std::cout << "Blocco NON Identificato\n" << std::endl;
    }
    std::cout << "Posizione finale del blocco: " << stampaVector(end_block_position).str() << "\n";

    return 0;
}