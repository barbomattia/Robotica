#include "TaskFunction.h"
#include "motion_planner/InverseKinematic.h"
#include "sensor_msgs/JointState.h"             // includo il messaggio per le gli stati dei joint
#include "std_msgs/Float64MultiArray.h"

void callback(const sensor_msgs::JointState::ConstPtr& msg, std::vector<double>* received_positions) {
    
    *received_positions = msg->position;
    std::stringstream jointposition;
    jointposition << "Joint Positions:\n";
    for (int i = 0; i < received_positions->size(); i++) {
        jointposition << "[ " << i << " ] " << (*received_positions)[i] <<"\n";
    }
    jointposition << "\n";
    std::cout << jointposition.str().c_str();
    
}

std::vector<double> ask_inverse_kinematic(ros::NodeHandle& n, double xef[3], double phief[3]){
 
    // client del service calculate_inverse_kinemaic gestito dal package motion_plan
    ros::ServiceClient service_client = n.serviceClient<motion_planner::InverseKinematic>("calculate_inverse_kinematics");
    // Crea un subscriber per il topic della joint state
    std::vector<double> received_positions;
    ros::Subscriber sub1 = n.subscribe<sensor_msgs::JointState>("/ur5/joint_states", 1, std::bind(callback, std::placeholders::_1, &received_positions));

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

    //inizializzo nella richiesta i parametri finali della configurazione end effector
    srv.request.xef[0]=xef[0];          srv.request.phief[0]=phief[0];
    srv.request.xef[1]=xef[1];          srv.request.phief[1]=phief[1];
    srv.request.xef[2]=xef[2];          srv.request.phief[2]=phief[2];

    
    /* TEST PRINT OF REQUEST MESSAGE 
    for (size_t i = 0; i < srv.request.jointstate.size(); ++i) {
        ROS_INFO("srv.request.jointstate[%zu]: %f", i, srv.request.jointstate[i]);
    }
    */

    if (service_client.call(srv)){
        std::stringstream q_received;
        for(int i=0; i<6; i++){
            q_received << srv.response.q[i] << " ";
        }
        printf("RESPONST SERVICE q: %s \n", q_received.str().c_str());
    } else {
        std::cout << "Failed to call service 'calculate_inverse_kinematics' \n";
        return {0.0};
    }

    std::vector<double> ret(std::begin( srv.response.q), std::end( srv.response.q));

    return ret;
  
}



void control_gazebo_arm(ros::NodeHandle& n, std::vector<double> q){

    /*
    ros::Publisher pub = n.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", 1);
    
    std_msgs::Float64MultiArray jointPositions;
    jointPositions.data.insert(jointPositions.data.end(), q.begin(), q.end()); 
        
    pub.publish(jointPositions);
    
    // Attendi un breve periodo per permettere al messaggio di essere pubblicato
    ros::Duration(1).sleep();  // Attendi per 1 secondo
    */

    ros::Publisher pub = n.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", 10);
    ros::Rate loop_rate(10);
    int count = 0;

    while (ros::ok()){
        std_msgs::Float64MultiArray jointPositions;
        jointPositions.data.insert(jointPositions.data.end(), q.begin(), q.end()); // Posizioni desiderate dei joint in radianti
        jointPositions.data.push_back(0);
        jointPositions.data.push_back(0);

        pub.publish(jointPositions);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    std::cout << " Messaggio Inviato a UR5 \n";

}