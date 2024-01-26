#include "TaskFunction.h"
#include "motion_planner/InverseKinematic.h"    // includo il messaggio per la richiesta di motion plan
#include <vision_planner/objectDetection.h>     // includo il messaggio per le richiesta di object detection
#include "sensor_msgs/JointState.h"             // includo il messaggio per le gli stati dei joint
#include "std_msgs/Float64MultiArray.h"

#define ARM_X 0.5
#define ARM_Y 0.35
#define ARM_Z 1.75


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*-------------------------------------------------------  FUNZIONI SECONDARIE ------------------------------------------------------------*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Sovraccaricamento dell'operatore <<
std::ostream& operator<<(std::ostream& os, const Block& block) {
    os << "name " << block.name;
    os << ", x [";
    for (int i = 0; i < 3; ++i) {
        os << block.x[i];
        if (i < 2) os << ", ";
    }
    os << "], phi [";
    for (int i = 0; i < 3; ++i) {
        os << block.phi[i];
        if (i < 2) os << ", ";
    }
    os << "] ";
    
    return os;
}


std::vector<Block> riordinaDatiBlocchi(const std::vector<double>& xBlocks, const std::vector<double>& phiBlocks, const std::vector<std::string>& nameBlocks) {

    std::vector<Block> ret;

    for(int i=0; i< nameBlocks.size(); i++) {
        Block b;
        b.name = nameBlocks[i];

        for(int j=0; j<3; j++){
            b.x[j] = xBlocks[(i*3)+j];
            b.phi[j] = xBlocks[(i*3)+j];
        }

        ret.push_back(b);

    }

    return ret;
}


std::stringstream stampaVector(const std::vector<double>& vec){
    std::stringstream ret;
    ret << " [ ";

    for (int i = 0; i < vec.size(); i++) {
        ret << vec[i] <<" ";
    }
    ret << "] ";

    return ret;
}

bool areVectorsEqual(const std::vector<double>& received_positions, const std::vector<double>& vec2) {
    
    // riordino received_positions nell'ordine corretto dei joint
    std::vector<double> vec = {received_positions[4], received_positions[3], received_positions[0], received_positions[5], received_positions[6], received_positions[7] };
    
    if (vec.size() != vec2.size()) {
        return false;
    }

    for (size_t i = 0; i < vec.size(); ++i) {
        if (std::abs(vec[i] - vec2[i]) > 0.1) {     // definisco treshold per non considerare altir errori
            return false; 
        }
    }

    return true; 
}

void callback(const sensor_msgs::JointState::ConstPtr& msg, std::vector<double>* received_positions) {
    
    *received_positions = msg->position;

    std::stringstream stampa;
    stampa << "Joint State Attuale: " << stampaVector(*received_positions).str() << "\n";
    std::cout << stampa.str().c_str();
    
}

void callback2(const sensor_msgs::JointState::ConstPtr& msg, std::vector<double>* received_positions) {
    
    *received_positions = msg->position;
    
}

bool isZero(const std::vector<double>& vettore) {
    for (double elemento : vettore) {
        if (elemento != 0.0) {
            return false; 
        }
    }
    return true; 
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*------------------------------------------------  FUNZIONI X MACCHINA A STATI  ----------------------------------------------------------*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

std::vector<Block> ask_object_detection(ros::NodeHandle& n){
    std::vector<Block> blocchi;
    ros::ServiceClient client = n.serviceClient<vision_planner::objectDetection>("object_detection");

    vision_planner::objectDetection srv;

    if (client.call(srv)){
        blocchi = riordinaDatiBlocchi(srv.response.xBlocks, srv.response.phiBlocks, srv.response.nameBlocks);
        std::cout<<"Blocchi Trovati \n";

        for (const auto& blocco : blocchi) {
            std::cout << "\tBlocco: " << blocco << std::endl;
        }
        std::cout<< std::endl;

        
    }else{
        // Se la chiamata non ha successo stmapo un errore
        std::cout << "Failed to call service 'object detection' \n";
    }

    return blocchi;
    
}




Eigen::MatrixXd ask_inverse_kinematic(ros::NodeHandle& n, double xef[3], double phief[3]){

    // client del service calculate_inverse_kinemaic gestito dal package motion_plan
    ros::ServiceClient service_client = n.serviceClient<motion_planner::InverseKinematic>("calculate_inverse_kinematics");
    // Crea un subscriber per il topic della joint state
    std::vector<double> received_positions;
    ros::Subscriber sub1 = n.subscribe<sensor_msgs::JointState>("/ur5/joint_states", 1, std::bind(callback, std::placeholders::_1, &received_positions));

    // Ciclo fino a quando non ricevi il messaggio joint state
    while (received_positions.empty()) {
        ros::spinOnce(); 
    }

    // Definisco la richiesta da inviare al motion planner
    motion_planner::InverseKinematic srv;  

    /* l'ordine degli 8 joint sono: 
    0 - elbow_joint, 1 - hand_1_joint, 2 - hand_2_joint, 3 - shoulder_lift_joint, 
    4- shoulder_pan_joint, 5 - wrist_1_joint, 6 - wrist_2_joint, 7- wrist_3_joint
    
    al motion_plan invio i joint in ordine: shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint */ 
    srv.request.jointstate.push_back(received_positions[4]);
    srv.request.jointstate.push_back(received_positions[3]);
    srv.request.jointstate.push_back(received_positions[0]);
    srv.request.jointstate.push_back(received_positions[5]);
    srv.request.jointstate.push_back(received_positions[6]);
    srv.request.jointstate.push_back(received_positions[7]);

    //inizializzo nella richiesta i parametri finali della configurazione end effector 
    //per le coordinate conferto le coordinate originali nel world frame alla coordinate del UR5 fram 
    srv.request.xef[0]=xef[0]-ARM_X;          srv.request.phief[0]=phief[0];
    srv.request.xef[1]=xef[1]-ARM_Y;          srv.request.phief[1]=phief[1];
    srv.request.xef[2]=xef[2]-ARM_Z;          srv.request.phief[2]=phief[2];

    
    /* TEST PRINT OF REQUEST MESSAGE 
    for (size_t i = 0; i < srv.request.jointstate.size(); ++i) {
        ROS_INFO("srv.request.jointstate[%zu]: %f", i, srv.request.jointstate[i]);
    }
    */

    Eigen::MatrixXd ret;
    
    if (service_client.call(srv)){

        // Se la chiamata ha successo 
        std::cout << "\nCONFIGURAZIONI di q: \n";

        ret.resize(srv.response.array_q.size() / 6, 6);
            
        std::stringstream q_received;
        for(int i=0; i < srv.response.array_q.size() / 6; i++){
            q_received << "\tq[" << i << "]: ";
            for(int j=0; j<6; j++){
                ret(i,j) = srv.response.array_q[(i*6)+j];
                q_received << srv.response.array_q[(i*6)+j] << "  ";
            }  
            q_received << "\n";
        }
        std::cout << q_received.str().c_str() << "\n";
        
    } else {
        // Se la chiamata non ha successo stmapo un errore
        std::cout << "Failed to call service 'calculate_inverse_kinematics' \n";
    }

    return ret;
  
}




void control_gazebo_arm(ros::NodeHandle& n, std::vector<double> q){
    
    ros::Publisher pub = n.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", 1);
    
    // inizializzo il messaggio per l'ur5 con la configurazione q dei joint che voglio raggiungere 
    std_msgs::Float64MultiArray jointPositions;
    jointPositions.data.insert(jointPositions.data.end(), q.begin(), q.end()); 
    jointPositions.data.push_back(0.0);     //inseirsco i valori  per il rasp1
    jointPositions.data.push_back(0.0);     //inseirsco i valori  per il rasp2

    // subscriber al topic che invia l'attuale configurazione dei joint dell'ur5
    std::vector<double> received_positions;  
    ros::Subscriber sub = n.subscribe<sensor_msgs::JointState>("/ur5/joint_states", 1, std::bind(callback2, std::placeholders::_1, &received_positions));

    // Ciclo fino a quando non ricevi il messaggio della configurazione attuale dei joint dell'ur5
    while (received_positions.empty()) {
        ros::spinOnce(); 
    }

    std::cout << "Braccio in Movimento:\n";
    ros::Rate loop_rate(10); 

    while(!areVectorsEqual(received_positions,q))
    {
        pub.publish(jointPositions);

        std::stringstream stampa;
        stampa << "Joint State: " << stampaVector(received_positions).str() << "\n";
        std::cout << stampa.str().c_str();

        ros::spinOnce(); 
        loop_rate.sleep();
    }
    

    std::cout << "Configurazione Raggiunta \n";

}




std::vector<double> define_end_position(std::string block){
    if(block == "X1-Y4-Z2")             return {0.9, 0.25, 0.865, 0.0, 0.0, 1.570795};
    if(block == "X1-Y4-Z1")             return {0.9, 0.3, 0.865, 0.0, 0.0, 1.570795};
    if(block == "X1-Y1-Z2")             return {0.8, 0.3, 0.865, 0.0, 0.0, 0.0};
    if(block == "X2-Y2-Z2")             return {0.92, 0.4, 0.865, 0.0, 0.0, 0.0};
    if(block == "X2-Y2-Z2-FILLET")      return {0.82, 0.4, 0.865, 0.0, 0.0, 0.0};
    if(block == "X1-Y3-Z2")             return {0.92, 0.5, 0.865, 0.0, 0.0, 1.570795};
    if(block == "X1-Y3-Z2-FILLET")      return {0.8, 0.5, 0.865, 0.0, 0.0, 1.570795};
    if(block == "X1-Y2-Z2-TWINFILLET")  return {0.95, 0.65, 0.865, 0.0, 0.0, 1.570795};
    if(block == "X1-Y2-Z2")             return {0.9, 0.65, 0.865, 0.0, 0.0, 0.0};
    if(block == "X1-Y2-Z1")             return {0.85, 0.65, 0.865, 0.0, 0.0, 1.570795};
    if(block == "X1-Y2-Z2-CHAMFER")     return {0.8, 0.65, 0.865, 0.0, 0.0, 1.570795};

    return {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    
    
}