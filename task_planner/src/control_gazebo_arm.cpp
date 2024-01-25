#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/JointState.h"             // includo il messaggio per le gli stati dei joint
#include <iostream>

/* JOINS ORDER 
1 -> primo joint ( spalla )
2 -> secondo joint ( spalla 2)
3 -> terzo joint ( gomito)
4 -> quarto joint ( polso 1 )
5 -> quinti joitn ( polso 2 )
6 -> sesto joitn ( polso 3 )
7 -> grip 1
8 -> grip 2

*/

std::stringstream stampaVector(const std::vector<double>& vec){
    std::stringstream ret;
    ret << " [ ";

    for (int i = 0; i < vec.size(); i++) {
        ret << vec[i] <<" ";
    }
    ret << "] ";

    return ret;
}

void callback2(const sensor_msgs::JointState::ConstPtr& msg, std::vector<double>* received_positions) {
    
    *received_positions = msg->position;
    
}

bool areVectorsEqualTo0(const std::vector<double>& vec1) {
    
    for (size_t i = 0; i < vec1.size(); ++i) {
        if (vec1[i] > 0.1) {     // definisco treshold per non considerare altir errori
            return false; 
        }
    }

    return true; 
}

int main(int argc, char **argv){
    ros::init(argc, argv, "control_gazebo_arm_task");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", 1);
    
    // inizializzo il messaggio per l'ur5 con la configurazione q dei joint che voglio raggiungere 
    std_msgs::Float64MultiArray jointPositions;
    jointPositions.data = {0,0,0,0,0,0,0,0};

    // subscriber al topic che invia l'attuale configurazione dei joint dell'ur5
    std::vector<double> received_positions;  
    ros::Subscriber sub = n.subscribe<sensor_msgs::JointState>("/ur5/joint_states", 1, std::bind(callback2, std::placeholders::_1, &received_positions));

    // Ciclo fino a quando non ricevi il messaggio della configurazione attuale dei joint dell'ur5
    while (received_positions.empty()) {
        ros::spinOnce(); 
    }

    
    std::cout << "Braccio in Movimento:\n";
    ros::Rate loop_rate(10); 

    while(!areVectorsEqualTo0(received_positions))
    {
        std::stringstream stampa;
        stampa << "Joint State Attuale: " << stampaVector(received_positions).str() << "\n";
        std::cout << stampa.str().c_str();

        pub.publish(jointPositions);

        ros::spinOnce(); 
        loop_rate.sleep();
    }
    

    std::cout << "Configurazione Raggiunta \n";

    return 0;
}