#include "ros/ros.h"
#include "TaskFunction.h"
#include <iostream>


int main(int argc, char **argv){
    ros::init(argc, argv, "state_machine_task");
    ros::NodeHandle n;
    
    ROS_INFO("--------------OBJECT DETECTION-----------------\n");
    std::vector<Block> blocchi = ask_object_detection(n);



    ROS_INFO("--------------INVERSE KINEMATIC----------------\n");
    double xefPROVA[3] = {0.2, 0.1, 0.1};
    double phiefPROVA[3] = {1.5, 0.0, 0.0};
    Eigen::MatrixXd q = ask_inverse_kinematic(n, xefPROVA, phiefPROVA);
   

    ROS_INFO("---------------MOTION OF THE ARM ----------------\n");
    
    /*
    for(int i = 0; i < q.rows(); i++){

        Eigen::MatrixXd qi = q.row(i);
        std::cout<<qi<<"\n";
        std::vector<double> qi_vector(qi.data(), qi.data() + qi.size());

        for (const auto& elem : qi_vector) {
            std::cout << elem << " ";
        }
        std::cout << std::endl;

        //control_gazebo_arm(n,qi_vector);
        std::cout <<"\n";

    }
    */


    return 0;
}