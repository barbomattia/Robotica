#include "ros/ros.h"
#include "motion_planner/InverseKinematic.h"
#include "sensor_msgs/JointState.h"             // includo il messaggio per le gli stati dei joint
#include <cstdlib>

bool received_message = false;
std::vector<double> received_positions;

void callback(const sensor_msgs::JointState::ConstPtr& msg) {
    received_positions = msg->position;
    ROS_INFO("Joint Positions:");
    for (int i = 0; i < received_positions.size(); i++) {
        ROS_INFO("[%d]: %f", i, received_positions[i]);
    }
}


int main(int argc, char **argv){

    ros::init(argc, argv, "ask_invers_kinematic_node");
    ros::NodeHandle n;

    // client del service calculate_inverse_kinemaic gestito dal package motion_plan
    ros::ServiceClient service_client = n.serviceClient<motion_planner::InverseKinematic>("calculate_inverse_kinematics");
    // Crea un subscriber per il topic della joint state
    ros::Subscriber sub1 = n.subscribe<sensor_msgs::JointState>("/ur5/joint_states", 1, callback);

    // Ciclo fino a quando non ricevi il messaggio joint state
    while (received_positions.empty()) {
        ros::spinOnce(); 
    }

    motion_planner::InverseKinematic srv;   
    /* l'ordine degli 8 joint sono: 0 - elbow_joint, 1 - hand_1_joint, 2 - hand_2_joint, 3 - shoulder_lift_joint, 
    4- shoulder_pan_joint, 5 - wrist_1_joint, 6 - wrist_2_joint, 7- wrist_3_joint
    
    al motion_plan invio i joint in ordine: shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint */ 
    srv.request.jointstate.push_back(received_positions[4]);
    srv.request.jointstate.push_back(received_positions[3]);
    srv.request.jointstate.push_back(received_positions[0]);
    srv.request.jointstate.push_back(received_positions[5]);
    srv.request.jointstate.push_back(received_positions[6]);
    srv.request.jointstate.push_back(received_positions[7]);
    
    
    for (size_t i = 0; i < srv.request.jointstate.size(); ++i) {
        ROS_INFO("srv.request.jointstate[%zu]: %f", i, srv.request.jointstate[i]);
    }

    if (service_client.call(srv)){
        ROS_INFO("q: %f", srv.response.q);
    } else {
        ROS_ERROR("Failed to call service");
        return 1;
    }
    
    return 0;
}
