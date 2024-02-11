/**
 * @file TaskFunction.h
 * @brief Header file containing the declarations of the functions necessary for the task planner
 * 
 */


#pragma once
#include <iostream>
#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include <iterator>
#include <functional>

#define END_EFFECTOR_WIDTH 0.5
#define ONE_WIDTH_BLOCK 0.31
#define TWO_WIDTH_BLOCK 0.63

#ifndef __TASKFUNCTION_H__
#define __TASKFUNCTION_H__


    /**
     * @brief Struttura che rappresenta un blocco di dati.
     * 
     * Questa struttura definisce un tipo di dato `Block` che contiene le sue coordinate (`x`), l'orientamento  (`phi`) ed il nome del blocco.
     */
    // definisco un tipo BLock per facilitare e rendere la gesstione dei dati più facile ed intuibile
    typedef struct {
        double x[3];        /**< Array delle coordinate x. */
        double phi[3];      /**< Array degli angoli phi. */
        std::string name;   /**< Nome del blocco. */
    } Block;


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
    // Dichiarazione dell'operatore <<
    std::ostream& operator<<(std::ostream& os, const Block& block);

    /**
     * @brief Controls the UR5 robot arm in Gazebo to reach the start position.
     * 
     * This function publishes joint positions to the UR5 robot arm's controller in Gazebo simulation to reach the start position
     * 
     * @param n A reference to the ROS NodeHandle object for communication.
     */
    void go_to_start_position(ros::NodeHandle& n);


    /**
     * @brief Requests object detection service and retrieves detected blocks.
     * 
     * This function calls the 'object_detection' service to detect blocks in the environment. It organizes the detected block data into a 
     * vector of Block objects and returns it.
     * 
     * @param n A reference to the ROS NodeHandle object for communication.
     * @return A vector of Block objects representing the detected blocks.
     */
    /* RICHIESTA OBJECT DETECTION
    La funzione richiede al nodo vision planner di eseguire un object detection per trovare tutti i blocci nella scena*/
    std::vector<Block> ask_object_detection(ros::NodeHandle& n);


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
     * @param title The name of the output file in wich the motion part will save the debug print.
     * @param first A boolean flag that indicate if it is the first ask for inverse kinematic.
     * @param grasp A boolean flag that indicate if the arm is grasping a block.
     * @return A matrix containing the computed joint configurations. Each row represents a set of joint values corresponding to a valid configuration.
     */
    /* RICHIESTA MOTION PLAN
    La funzione richiede al nodo motion di eseguire il motion plan per raggiungere la configurazione richiesta: [xef, phief]
    Essa ritorna le configurazione q */
    Eigen::MatrixXd ask_inverse_kinematic(ros::NodeHandle& n, double xef[3], double phief[3], std::string title, bool first, bool grasp);


    /* RICHIESTA MOVIMENTO BRACCIO
    La funzione richiede a gazebo di muovere il braccio robotico usando la configurazione dei joint q passata*/


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
    void control_gazebo_arm(ros::NodeHandle& n, std::vector<double> q);


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
    void control_gazebo_arm_2(ros::NodeHandle& n, Eigen::MatrixXd q, bool goingBack, bool grasp);


    /* DEFINIZIONE POSIZIONE ORDINATA BLOCCO
    La funzione ritorna la posizione in cui mettere il blocco */
    std::vector<double> define_end_position(std::string block);


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
    /* FUNZIONI SECONDARIE */
    bool isZero(const std::vector<double>& vettore);

    /**
     * @brief Calculates the width adjustment for the gripper based on the provided block name.
     * 
     * This function determines the gripper width adjustment according to the specified block name. If the block name matches any in the 
     * OneWidth vector, it computes half the difference between the end effector width and the width of a single-width block. If it matches any in 
     * the TwoWidth vector, it computes half the difference between the end effector width and the width of a double-width block. 
     * If the provided block name does not match any predefined block names, it prints a message indicating an unrecognized block name.
     * 
     * @param blockName The name of the block for which the gripper width adjustment is calculated.
     * 
     * @return The width adjustment for the gripper.
     * If the block name is recognized: for single-width blocks: half the difference between the end effector width and the width of a 
     * single-width block. For double-width blocks: half the difference between the end effector widt and the width of a double-width block.
     * If the block name is unrecognized returns 0.
     */
    double Gripper(std::string blockName);


    /**
     * @brief Menage the action of graping.
     * 
     * This function menage the action of grasping, in both taken and release event. It open or close the gripper in case of taken or release action 
     * and request to the motion part the trajectory to reach the block.
     * 
     * @param n A reference to the ROS NodeHandle object for communication.
     * @param xef An array containing the x, y, z coordinates of the desired end effector position.
     * @param phief An array containing the roll, pitch, yaw angles (in radians) of the desired end effector orientation.
     * @param take A boolean flag that indicate if it is a teken or release action.
     * @param blockName The name of the block for which the gripper width adjustment is calculated.
     * @param title The name of the output file in wich the motion part will save the debug print.
     * 
     */
    void grasp(ros::NodeHandle& n, double xef[3], double phief[3], bool take, std::string blockName, std::string title);

    std::stringstream stampaVector(const std::vector<double>& vec);


#endif
