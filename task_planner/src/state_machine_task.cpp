#include "ros/ros.h"
#include "TaskFunction.h"
#include <iostream>


bool ordering_block(ros::NodeHandle& n, Block& blocco){

    std::cout << "\n///////////////////////////////// BLOCK " << blocco.name << " //////////////////////////////////////////\n";

    std::cout << "\n\t -> ASK MOTION PLAN \n";
    Eigen::MatrixXd q = ask_inverse_kinematic(n, blocco.x, blocco.phi);

    /*for(int i = 0; i < q.rows(); i++){
            Eigen::MatrixXd qi = q.row(i);           
        std::cout<< "["<< i << "]" << qi <<"\n";
    }*/

    if(q.rows() < 100){
        std::cout << "\n\t -> BLOCK NON REACABLE \n";
        return false;
    }

    std::cout << "\n\t -> REACHING THE BLOCK \n";
    
    
    if(q.rows()==200){
        std::cout << "\n\t -> STEP 1 ";
        control_gazebo_arm_2(n,q.topRows(100),false);

        std::cout << "\n\t -> STEP 2  \n";
        control_gazebo_arm_2(n,q.bottomRows(100),false);

    }else if(q.rows()==300){
        std::cout << "\n\t\t STEP 1 ";
        control_gazebo_arm_2(n,q.topRows(100),false);

        std::cout << "\n\t\t STEP 2  \n";
        control_gazebo_arm_2(n,q.block(100, 0, 100, 6),false);
        
        std::cout << "\n\t\t STEP 3  \n";
        control_gazebo_arm_2(n,q.bottomRows(100),false);

    }else{
        control_gazebo_arm_2(n,q,false);
    }
    
    

    /* std::cout << "\n ----------------------------- GOING BACK HOME POSITION ----------------------------- \n";
    control_gazebo_arm_2(n,q,true); */
    
    std::cout << "\n\t -> GRASP THE BLOCK \n";

    std::cout << "\n\t -> PUT THE BLOCK IN ORDER  \n";
    std::vector<double> end_block_position = define_end_position(blocco.name);

    // Creare due array di tipo double[3]
    double end_block_position_x[3];
    double end_block_position_phi[3];

    // Copia i primi 3 elementi di end_block_position in end_block_position_x
    std::copy(end_block_position.begin(), end_block_position.begin() + 3, end_block_position_x);
    
    // Copia gli elementi rimanenti di end_block_position in end_block_position_phi
    std::copy(end_block_position.begin() + 3, end_block_position.end(), end_block_position_phi);
      
    if (isZero(end_block_position)) {
        std::cout << "\tBlocco NON Identificato\n" << std::endl;
        return false;
    }

    std::cout << "\t    Posizione Ordinata del blocco: \n";

    std::cout << "\t\t  x: ";
    for (double d : end_block_position_x) { 
        std::cout << d << " ";
    }
    std::cout << std::endl;

    std::cout << "\t\t  phi: ";
    for (double d : end_block_position_phi) {
        std::cout << d << " ";
    }
    std::cout << std::endl;

    Eigen::MatrixXd q_end = ask_inverse_kinematic(n, end_block_position_x, end_block_position_phi);
    std::cout << std::endl << "\t    Reaching the order coordinate\n" <<std::endl;
    control_gazebo_arm_2(n,q_end,false);

    return true;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "state_machine_task");
    ros::NodeHandle n;

    bool last_one_bloc = false;

    std::cout << "\n\n&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& START &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&\n\n";

    std::cout << "----------------------------- OBJECT DETECTION ---------------------------------\n";
    std::vector<Block> blocchi = ask_object_detection(n);           // ottengo tutti i blocchi trovati
    /* Block blocco;                                                   // dichiaro il blocco che voglio ordinare    
    blocco.x[0] = 0.24; blocco.x[1] = 0.69; blocco.x[2] = 1.1;
    blocco.phi[0] = 0; blocco.phi[1] = 0; blocco.phi[2] = 0;
    blocco.name = "X2-Y2-Z2"; */
                                
    for(Block& blocco : blocchi){
        ordering_block(n, blocco);
    }
              

    return 0;
}