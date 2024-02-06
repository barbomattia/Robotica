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


bool areVectorsEqual(const std::vector<double>& received_positions, const std::vector<double>& vec2) {
    // riordino received_positions nell'ordine corretto dei joint
    std::vector<double> vec = {received_positions[4], received_positions[3], received_positions[0], received_positions[5], received_positions[6], received_positions[7] };
    std::vector<double> vec1 = {vec2[0], vec2[1], vec2[2], vec2[3], vec2[4], vec2[5] };

    if (vec.size() != vec1.size()) {
        std::cout<<"DIMENSIONI DIFFERENTI" << std::endl;
        return false;
    }

    for (size_t i = 0; i < vec.size(); ++i) {
        if (std::abs(vec[i] - vec1[i]) > 0.01) {     // definisco treshold per non considerare altir errori
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
    jointPositions.data = {-0.320096, -0.780249, -2.560807, -1.630513, -1.570495, 3.491099, 0,0};
    // subscriber al topic che invia l'attuale configurazione dei joint dell'ur5
    std::vector<double> received_positions;  
    ros::Subscriber sub = n.subscribe<sensor_msgs::JointState>("/ur5/joint_states", 1, std::bind(callback2, std::placeholders::_1, &received_positions));

    // Ciclo fino a quando non ricevi il messaggio della configurazione attuale dei joint dell'ur5
    while (received_positions.empty()) {
        ros::spinOnce(); 
    }

    
    std::cout << "Braccio in Movimento:\n";
    ros::Rate loop_rate(200); 

    while(!areVectorsEqual(received_positions, jointPositions.data))
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