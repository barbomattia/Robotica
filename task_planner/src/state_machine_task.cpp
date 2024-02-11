/**
 * @file state_machine_task.cpp
 * @brief C++ file that implement the task planner core
 * 
 */


#include "ros/ros.h"
#include "TaskFunction.h"
#include <iostream>



/**
 * @brief Orders the robot arm to manipulate a block.
 * 
 * This function orchestrates the motion planning and execution for manipulating a block. It first calculates the inverse kinematics 
 * to reach the block's initial position. If the block is reachable, it proceeds to move the arm to reach the block, potentially 
 * in multiple steps. Then, it grasps the block and puts it in the desired order.
 * 
 * @param n A reference to the ROS node handle.
 * @param blocco The block object representing the block to be manipulated.
 * @param i Integer that indicate the iteration of the loop that call this function.
 * 
 * @return A boolean value indicating whether the operation was successful, if it's true, the block was successfully manipulated, else the 
 * block was not reachable or other errors occurred.
 * 
 */
bool ordering_block(ros::NodeHandle& n, Block& blocco, int i){

    std::cout << "\n///////////////////////////////// BLOCK " << blocco.name << " //////////////////////////////////////////\n";

    std::cout << "\n\t -> ASK MOTION PLAN \n";
    std::string title = "Reach " + blocco.name;

    Eigen::MatrixXd q;

    if(i==0){
        q = ask_inverse_kinematic(n, blocco.x, blocco.phi, title, true, false);
    }else{
        q = ask_inverse_kinematic(n, blocco.x, blocco.phi, title, false, false);
    }
    

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
        std::cout << "\n\t -> STEP 1 "; std::cout.flush();
        control_gazebo_arm_2(n, q.topRows(100), false, false);

        std::cout << "\n\t -> STEP 2  \n"; std::cout.flush();
        control_gazebo_arm_2(n,q.bottomRows(100),false, false);

    }else if(q.rows()==300){
        std::cout << "\n\t\t STEP 1 "; std::cout.flush();
        control_gazebo_arm_2(n,q.topRows(100),false, false);

        std::cout << "\n\t\t STEP 2  \n"; std::cout.flush();
        control_gazebo_arm_2(n,q.block(100, 0, 100, 6),false, false);
        
        std::cout << "\n\t\t STEP 3  \n"; std::cout.flush();
        control_gazebo_arm_2(n,q.bottomRows(100),false, false);

    }else{
        control_gazebo_arm_2(n,q,false, false);
    }
    
    

    /* std::cout << "\n ----------------------------- GOING BACK HOME POSITION ----------------------------- \n";
    control_gazebo_arm_2(n,q,true); */
    
    std::cout << "\n\t -> GRASP THE BLOCK \n";
    title = "Grasp " + blocco.name;
    grasp(n, blocco.x, blocco.phi, true, blocco.name, title);
    std::cout << "\n\t     end grasping\n"; std::cout.flush();

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

    /* Se il braccio ha raggiunto il blocco in più step segue a ritroso la traiettoria fino al primo 
    punto intermedio e poi da li calcola la traiettoria al punto ordinato. Altrimenti calcola direttamente la traiettoria */
    if(q.rows() > 100){
        std::cout << "\t    Going back to fiirst mid point"; std::cout.flush();
        if(q.rows()==200){
            control_gazebo_arm_2(n,q.bottomRows(100),true, true);

        }else{
            control_gazebo_arm_2(n,q.bottomRows(100),true, true);
            control_gazebo_arm_2(n,q.block(100, 0, 100, 6),true, true);
                
        }

    }

    title = "Ordering " + blocco.name;
    std::cout << std::endl << "\t    Reaching the order coordinate\n" <<std::endl; std::cout.flush();
    Eigen::MatrixXd q_end = ask_inverse_kinematic(n, end_block_position_x, end_block_position_phi, title, false, false);
    control_gazebo_arm_2(n,q_end,false, true);

    std::cout << "\n\t -> POSE THE BLOCK: "; std::cout.flush();
    title = "Pose " + blocco.name;
    grasp(n, end_block_position_x, end_block_position_phi, false, blocco.name, title);
    std::cout << "\n\t     end grasping\n"; std::cout.flush();
    
    
    return true;
}

/**
 * @brief Define the behavior of the task planner node.
 * 
 */
int main(int argc, char **argv){
    ros::init(argc, argv, "state_machine_task");
    ros::NodeHandle n;

    bool last_one_bloc = false;

    std::cout << "\n\n&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& START &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&\n\n";

    std::cout << "----------------------------- GO TO START POSITION ---------------------------------\n";
    go_to_start_position(n);

    std::cout << "\n----------------------------- OBJECT DETECTION ---------------------------------\n";
    std::vector<Block> blocchi = ask_object_detection(n);           // ottengo tutti i blocchi trovati
    /* Block blocco;                                                   // dichiaro il blocco che voglio ordinare    
    blocco.x[0] = 0.24; blocco.x[1] = 0.69; blocco.x[2] = 1.1;
    blocco.phi[0] = 0; blocco.phi[1] = 0; blocco.phi[2] = 0;
    blocco.name = "X2-Y2-Z2"; */
                                
    for (int i = 0; i < blocchi.size(); ++i) {
        ordering_block(n, blocchi[i], i);
    }
              

    return 0;
}