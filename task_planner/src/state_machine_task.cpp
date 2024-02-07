#include "ros/ros.h"
#include "TaskFunction.h"
#include <iostream>


bool ordering_block(ros::NodeHandle& n, Block& blocco){

    std::cout << "\n///////////////////////////////// BLOCK " << blocco.name << " //////////////////////////////////////////\n";

    std::cout << "\n\t -> ASK MOTION PLAN \n";
    Eigen::MatrixXd q = ask_inverse_kinematic(n, blocco.x, blocco.phi);


    std::cout << "\n\t -> REACHING THE BLOCK \n";
    
    /* for(int i = 0; i < q.rows(); i++){
        Eigen::MatrixXd qi = q.row(i);          // std::cout<<qi<<"\n";
        std::vector<double> qi_vector(qi.data(), qi.data() + qi.size());
        for (const auto& elem : qi_vector) {       std::cout << elem << " ";}
        std::cout << std::endl;  control_gazebo_arm(n,qi_vector);  std::cout <<"\n";
    } */
    
    control_gazebo_arm_2(n,q,false);

    
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

    std::cout << "\tPosizione Ordinata del blocco: \n";

    std::cout << "\t\tx: ";
    for (double d : end_block_position_x) { 
        std::cout << d << " ";
    }
    std::cout << std::endl;

    std::cout << "\t\tphi: ";
    for (double d : end_block_position_phi) {
        std::cout << d << " ";
    }
    std::cout << std::endl;

    Eigen::MatrixXd q_end = ask_inverse_kinematic(n, end_block_position_x, end_block_position_phi);
    std::cout << std::endl << "\tReaching the order coordinate\n" <<std::endl;
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