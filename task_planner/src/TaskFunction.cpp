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


/**
 * @brief Overloaded insertion operator for outputting Block information.
 * 
 * This overload allows the insertion operator '<<' to be used with a Block object for outputting its information to an output stream.
 * 
 * @param os The output stream to which the Block information will be written.
 * @param block The Block object whose information will be outputted.
 * 
 * @return A reference to the output stream 'os'.
 */
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


/**
 * @brief Orders the data of blocks into a vector of Block objects.
 * 
 * This function takes three vectors representing the x coordinates, phi angles, and names of blocks and organizes them into a vector of
 * Block objects. Each Block object contains the x coordinates, phi angles, and name of a block.
 * 
 * @param xBlocks A vector containing the x coordinates of blocks.
 * @param phiBlocks A vector containing the phi angles of blocks.
 * @param nameBlocks A vector containing the names of blocks.
 * 
 * @return A vector of Block objects organized with the data of the input blocks.
 */
std::vector<Block> riordinaDatiBlocchi(const std::vector<double>& xBlocks, const std::vector<double>& phiBlocks, const std::vector<std::string>& nameBlocks) {

    std::vector<Block> ret;

    for(int i=0; i< nameBlocks.size(); i++) {
        Block b;
        b.name = nameBlocks[i];

        for(int j=0; j<3; j++){
            b.x[j] = xBlocks[(i*3)+j];
            b.phi[j] = phiBlocks[(i*3)+j];
        }

        ret.push_back(b);

    }

    return ret;
}


/**
 * @brief Converts a vector of double values to a formatted string stream.
 * 
 * This function takes a vector of double values and converts it into a formatted string stream. Each element of the vector is appended 
 * to the string stream, separated by spaces and enclosed within square brackets.
 * 
 * @param vec The vector of double values to be converted.
 * 
 * @return A stringstream containing the formatted representation of the input vector.
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


/**
 * @brief Compares two vectors of double values for equality within a defined threshold.
 * 
 * This function compares two vectors of double values for equality within a defined threshold. It reorders the elements of the first 
 * vector 'received_positions' to match the correct orderof joint positions. Then, it checks if the two vectors have the same size. 
 * If their sizes are different, it returns false. Otherwise, it iterates through the elements of the vectors and compares them. 
 * If the absolute difference between corresponding elements exceeds the defined threshold, it returns false. Otherwise, it returns true, 
 * indicating that the vectors are equal within the specified threshold.
 * 
 * @param received_positions The first vector of double values to be compared.
 * @param vec2 The second vector of double values to be compared.
 * 
 * @return A boolean value indicating whether the two vectors are equal within the defined threshold.If it's true, the vectors are equal 
 * within the threshold, else the vectors are not equal within the threshold.
 * 
 */
bool areVectorsEqual(const std::vector<double>& received_positions, const std::vector<double>& vec2) {
    // riordino received_positions nell'ordine corretto dei joint
    std::vector<double> vec = {received_positions[4], received_positions[3], received_positions[0], received_positions[5], received_positions[6], received_positions[7] };
    
    if (vec.size() != vec2.size()) {
        std::cout<<"DIMENSIONI DIFFERENTI" << std::endl;
        return false;
    }

    for (size_t i = 0; i < vec.size(); ++i) {
        if (std::abs(vec[i] - vec2[i]) > 0.01) {     // definisco treshold per non considerare altir errori
            return false; 
        }
    }

    return true; 
}


/**
 * @brief Callback function for processing joint state messages.
 * 
 * This function is called whenever a new message of type sensor_msgs::JointState is received. It updates the received_positions 
 * vector with the joint positions from the message. Additionally, it creates a stringstream object to construct a message for logging purposes.
 * 
 * @param msg The received JointState message containing joint positions.
 * @param received_positions A pointer to the vector where the received joint positions will be stored.
 */
void callback(const sensor_msgs::JointState::ConstPtr& msg, std::vector<double>* received_positions) {
    
    *received_positions = msg->position;

    std::stringstream stampa;
    // stampa << "Joint State Attuale: " << stampaVector(*received_positions).str() << "\n";
    std::cout << stampa.str().c_str();
    
}


/**
 * @brief Callback function for handling incoming joint state messages.
 * 
 * This function is called every time a new message is received on the 'jointState' topic. It extracts the joint positions from the 
 * received message and stores them in the provided vector.
 * 
 * @param msg Pointer to the received joint state message.
 * @param received_positions Pointer to the vector where the received joint positions will be stored.
 */
void callback2(const sensor_msgs::JointState::ConstPtr& msg, std::vector<double>* received_positions) {
    
    *received_positions = msg->position;
    
}


/**
 * @brief Checks if all elements of a vector are zero.
 * 
 * This function iterates through the elements of the input vector and checks if each element is equal to zero. If any element is non-zero, 
 * the function returns false, indicating that the vector is not composed entirely of zeros. Otherwise, if all elements are zero, 
 * the function returns true.
 * 
 * @param vettore The vector of double values to be checked.
 * @return true if all elements of the vector are zero, false otherwise.
 */
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


/**
 * @brief Requests object detection service and retrieves detected blocks.
 * 
 * This function calls the 'object_detection' service to detect blocks in the environment. It organizes the detected block data into a 
 * vector of Block objects and returns it.
 * 
 * @param n A reference to the ROS NodeHandle object for communication.
 * @return A vector of Block objects representing the detected blocks.
 */
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



/**
 * @brief Requests inverse kinematics calculation service to compute joint configurations.
 * 
 * This function calls the 'calculate_inverse_kinematics' service provided by the motion_planner package to compute the joint configurations 
 * required to reach a desired end effector position and orientation. It subscribes to the joint states topic to retrieve the current robot 
 * joint positions.
 * 
 * @param n A reference to the ROS NodeHandle object for communication.
 * @param xef An array containing the x, y, z coordinates of the desired end effector position.
 * @param phief An array containing the roll, pitch, yaw angles (in radians) of the desired end effector orientation.
 * @return A matrix containing the computed joint configurations. Each row represents a set of joint values corresponding to a valid configuration.
 */
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
    srv.request.xef[0]=xef[0]-ARM_X;            srv.request.phief[0]=phief[0];
    srv.request.xef[1]=ARM_Y - xef[1];            srv.request.phief[1]=phief[1];
    srv.request.xef[2]=ARM_Z - xef[2];          srv.request.phief[2]=phief[2];

    
    /* TEST PRINT OF REQUEST MESSAGE 
    for (size_t i = 0; i < srv.request.jointstate.size(); ++i) {
        ROS_INFO("srv.request.jointstate[%zu]: %f", i, srv.request.jointstate[i]);
    }
    */

    Eigen::MatrixXd ret;
    
    if (service_client.call(srv)){

        // Se la chiamata ha successo 
        std::cout << "\n\t    CONFIGURAZIONI di q CALCOLATE: ";
        if(srv.response.two_step) std::cout << "SOLUZIONE a 2 Step";
        std::cout << std::endl;

        ret.resize(srv.response.array_q.size() / 6, 6);     
        for(int i=0; i < srv.response.array_q.size() / 6; i++){
            for(int j=0; j<6; j++){
                ret(i,j) = srv.response.array_q[(i*6)+j];
            }  
        }
        
        
    } else {
        // Se la chiamata non ha successo stmapo un errore
        std::cout << "\n\t   Failed to call service 'calculate_inverse_kinematics' \n";
    }

    return ret;
  
}



/**
 * @brief Controls the UR5 robot arm in Gazebo simulation.
 * 
 * This function publishes joint positions to the UR5 robot arm's controller in Gazebo simulation. It initializes the message with 
 * the desired joint positions and publishes it to the appropriate topic. It also subscribes to the joint states topic to receive the 
 * current joint positions of the UR5 arm. The function continuously publishes the desired joint positions until the current joint positions 
 * match the desired ones, indicating that the arm has reached the desired configuration.
 * 
 * @param n A reference to the ROS NodeHandle object for communication.
 * @param q A vector containing the desired joint positions for the UR5 arm.
 */
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
    ros::Rate loop_rate(25); 

    while(!areVectorsEqual(received_positions,q))
    {
        pub.publish(jointPositions);

        /* std::stringstream stampa;
        stampa << "Joint State: " << stampaVector(received_positions).str() << "\n";
        std::cout << std::endl << stampa.str().c_str(); */

        ros::spinOnce(); 
        loop_rate.sleep();
    }
    

    std::cout << "\tConfigurazione Raggiunta \n";

}


/**
 * @brief Controls the UR5 robot arm in Gazebo simulation using a trajectory.
 * 
 * This function publishes joint positions to the UR5 robot arm's controller in Gazebo simulation to follow a trajectory specified by the 
 * input matrix `q`. It initializes messages with the desired joint positions at each step of the trajectory and publishes them to the 
 * appropriate topic. The function can control the arm to follow the trajectory in the forward direction or move back along the trajectory, 
 * depending on the value of the `goingBack` parameter.
 * 
 * @param n A reference to the ROS NodeHandle object for communication.
 * @param q The matrix representing the trajectory, where each row contains the desired joint positions for a specific step of the trajectory.
 * @param goingBack A boolean flag indicating whether to move back along the trajectory (true) or follow it in the forward direction (false).
 */
void control_gazebo_arm_2(ros::NodeHandle& n, Eigen::MatrixXd q, bool goingBack){

    ros::Publisher pub = n.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", 1);
    std::vector<std_msgs::Float64MultiArray> jointPositions(q.rows());

    for(int i = 0; i < q.rows(); i++){
        Eigen::MatrixXd qi;

        if(goingBack){ qi = q.row(q.rows()-1-i); }
        else { qi = q.row(i); }
        
        std::vector<double> qi_vector(qi.data(), qi.data() + qi.size());
        
        std_msgs::Float64MultiArray jointPositionTemp;
        jointPositionTemp.data.insert(jointPositionTemp.data.end(), qi_vector.begin(), qi_vector.end()); 
        jointPositionTemp.data.push_back(0.0);     //inserisco i valori per il rasp1
        jointPositionTemp.data.push_back(0.0);     //inserisco i valori per il rasp2

        jointPositions[i] = jointPositionTemp;

    }

    ros::Rate loop_rate(50); 
    int count = 0;
   
    while (count != q.rows()) {
        pub.publish(jointPositions[count]);
        count++;
        ros::spinOnce(); 
        loop_rate.sleep();
    }

}



std::vector<double> define_end_position(std::string block){
    if(block == "X1-Y4-Z2")             return {0.9, 0.25, 1.1, 0.0, 0.0, 1.570795};
    if(block == "X1-Y4-Z1")             return {0.9, 0.3, 1.1, 0.0, 0.0, 1.570795};
    if(block == "X1-Y1-Z2")             return {0.8, 0.3, 1.1, 0.0, 0.0, 0.0};
    if(block == "X2-Y2-Z2")             return {0.92, 0.4, 1.1, 0.0, 0.0, 0.0};
    if(block == "X2-Y2-Z2-FILLET")      return {0.82, 0.4, 1.1, 0.0, 0.0, 0.0};
    if(block == "X1-Y3-Z2")             return {0.92, 0.5, 1.1, 0.0, 0.0, 1.570795};
    if(block == "X1-Y3-Z2-FILLET")      return {0.8, 0.5, 1.1, 0.0, 0.0, 1.570795};
    if(block == "X1-Y2-Z2-TWINFILLET")  return {0.95, 0.65, 1.1, 0.0, 0.0, 1.570795};
    if(block == "X1-Y2-Z2")             return {0.9, 0.65, 1.1, 0.0, 0.0, 0.0};
    if(block == "X1-Y2-Z1")             return {0.85, 0.65, 1.1, 0.0, 0.0, 1.570795};
    if(block == "X1-Y2-Z2-CHAMFER")     return {0.8, 0.65, 1.1, 0.0, 0.0, 1.570795};
    if(block == "X1-Y1-Z1")             return {0.75, 0.65, 1.1, 0.0, 0.0, 1.570795};


    return {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    
    
}