/**
 * @file Kinematic.h
 * @author Mattia Barborini, Matteo Grisenti
 * @brief Header file containing the declarations of the functions necessary for the arm movement
 * @date 2024-02-07
 * 
 */


#pragma once
#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include <functional>

/**
 * @brief 
 * 
 */
#define ARM_X 0.5
#define ARM_Y 0.35
#define ARM_Z 1.75

#define END_EFFECTOR_WIDTH 0.05
#define ONE_WIDTH_BLOCK 0.031
#define TWO_WIDTH_BLOCK 0.063

#define DER_H 0.04
#define K0 0.01

#ifndef __KINEMATIC_H__
#define __KINEMATIC_H__



    /**
     * @brief Struct containing the arm transformation matrices
     * 
     */
    struct TransformationMatrices { Eigen::Matrix4d T10, T21, T32, T43, T54, T65, T60; };

    /**
     * @brief Struct containing the return values ​​of the CinematicaDiretta function 
     * 
     */
    //CONFIGURAZIONE END EFFECTOR -----------------------------------------------------------------------------------------------
    struct CinDir{
        Eigen::Vector3d pe;     //POSIZIONE END EFFECTOR
        Eigen::Matrix3d Re;     //ROTAZIONE END EFFECTOR
    };

    /**
     * @brief Struct containing the return values ​​of the getFirstColumnWithoutNaN function
     * 
     */
    struct NaNColumn{
        Eigen::VectorXd configurazione;
        bool isNaN;
    };

    /**
     * @brief Struct containing the x, y and z coordinates of a point in the world frame
     * 
     */
    struct Point{
        double x;
        double y;
        double z;
    };


    /**
     * @brief Return a random int value between the lower and the higher values
     * 
     * @param min lower value
     * @param max higher value
     * @return int 
     */
    int random(int min, int max);

    /**
     * @brief Computes the forward kinematics for a robotic arm using Denavit-Hartenberg parameters.
     * 
     * This function calculates the forward kinematics of a robotic arm given the joint angles and a scaling factor.
     * It utilizes Denavit-Hartenberg parameters to define the robot's kinematic chain.
     * 
     * @param Th The joint angles vector.
     * @param scaleFactor The scaling factor applied to the Denavit-Hartenberg parameters.
     * @return A CinDir object containing the position and orientation of the end effector.
     * 
     * The function initializes the DH parameters A, D, and Alpha of the robot arm and computes the transformation matrices
     * for each link using the provided joint angles and DH parameters. It then calculates the final transformation matrix
     * representing the end effector's position and orientation.
     * 
     * The transformation matrices are computed using the provided joint angles, DH parameters, and a given scaling factor.
     * 
     * @note This function assumes rotational joints only.
     * @note The joint angles vector Th should have a size of 6, representing the angles for each joint.
     */
    CinDir CinematicaDiretta(const Eigen::VectorXd& Th, double scaleFactor);


    /**
     * @brief Computes the inverse kinematics for a robotic arm given the end effector position and orientation.
     * 
     * This function calculates the inverse kinematics of a robotic arm to determine the joint angles required
     * to achieve a specific end effector position and orientation.
     * 
     * @param p60 The position vector of the end effector.
     * @param R60 The orientation matrix of the end effector.
     * @param scaleFactor The scaling factor applied to the Denavit-Hartenberg parameters.
     * @return A matrix containing the joint angles configuration for the robotic arm.
     * 
     * The function initializes the DH parameters A, D, and Alpha of the robot arm and computes the joint angles
     * required to achieve the given end effector position and orientation.
     * 
     * The joint angles are calculated based on the provided end effector position, orientation, and scaling factor.
     * 
     * @note This function assumes rotational joints only.
     * @note The input position vector p60 should have three elements representing x, y, and z coordinates respectively.
     * @note The input orientation matrix R60 should be a 3x3 rotation matrix representing the end effector's orientation.
     * @note The returned matrix Th contains the joint angles for the robot arm in an 6x8 configuration, each column represents a specific 
     *       joint angle configuration, and each row represents a specific joint angle for the respective column.
     */
    Eigen::MatrixXd cinematicaInversa(Eigen::Vector3d p60, Eigen::Matrix3d R60, double scaleFactor);


    /**
     * @brief Computes the Jacobian matrix for the UR5 robotic arm.
     * 
     * This function calculates the Jacobian matrix for the UR5 robotic arm given the joint angles configuration Th
     * and a scaling factor scaleFactor applied to the Denavit-Hartenberg parameters.
     * 
     * @param Th The joint angles configuration of the UR5 robotic arm.
     * @param scaleFactor The scaling factor applied to the Denavit-Hartenberg parameters.
     * @return The Jacobian matrix for the UR5 robotic arm.
     * 
     * The Jacobian matrix provides a mapping between the joint velocities and the end effector's linear and angular velocities.
     * 
     * @note This function assumes rotational joints only.
     * @note The input joint angles configuration Th should be a vector of length 6 representing the joint angles.
     * @note The returned Jacobian matrix J is a 6x6 matrix, each column represents the linear and angular velocities of the end effector 
     *       corresponding to the respective joint, and each row represents a specific component of the end effector's velocity.
     */
    Eigen::MatrixXd ur5Jac(const Eigen::VectorXd& Th, double scaleFactor);


    //FUNZIONE per CALCOLO CONIUGATA di un QUATERNIONE -------------------------------------------------------------------------------------
    Eigen::Quaterniond quatconj(const Eigen::Quaterniond& q);
       


    //FUNZIONE per CALCOLO PRODOTTO tra QUATERNIONI --------------------------------------------------------------------------------------
    Eigen::Quaterniond quatmultiply(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2);
      

    //FUNZIONE che RITORNA la PARTE VETTORIALE di un QUATERNION --------------------------------------------------------------------------
    Eigen::Vector3d parts(const Eigen::Quaterniond& q); 
        
    /**
    * @brief Computes the damped pseudo-inverse of a matrix using the Moore-Penrose inverse.
    * 
    * This function calculates the damped pseudo-inverse of a matrix J using the Moore-Penrose inverse,
    * with an additional damping factor added to the diagonal elements of the product J * J^T before inversion.
    * 
    * @param J The matrix for which the damped pseudo-inverse is computed.
    * @return The damped pseudo-inverse of the input matrix J.
    * 
    * The damped pseudo-inverse is useful for computing the pseudo-inverse of matrices that are close to singular,
    * where regularization helps stabilize the inversion process.
    * 
    * @note This function assumes that the input matrix J is not necessarily square and may be of full rank or rank-deficient.
    * @note The damping factor is a small positive scalar added to the diagonal elements of J * J^T before inversion.
    * @note The returned pseudo-inverse matrix has the dimensions of J^T, i.e., J.columns() x J.rows().
    */
    // FUNZIONE per CALCOLO della PSEUDOINVERSA DESTRA SMORZATA
    Eigen::MatrixXd dampedPseudoInverse(const Eigen::MatrixXd& J);


    /**
     * @brief Computes the inverse differential kinematic control for a UR5 robotic arm with quaternion representation.
     * 
     * This function calculates the inverse differential kinematic control for a UR5 robotic arm with quaternion representation.
     * It computes the joint velocity vector dotQ required to move the end effector from its current configuration xe to the desired configuration xd.
     * 
     * @param q The vector of current joint positions.
     * @param xe The current configuration of the end effector.
     * @param xd The desired configuration of the end effector.
     * @param vd The desired linear velocity of the end effector.
     * @param omegad The desired angular velocity of the end effector.
     * @param qe The current quaternion configuration of the end effector.
     * @param qd The desired quaternion configuration of the end effector.
     * @param Kp The linear error matrix.
     * @param Kq The quaternion error matrix.
     * @param scaleFactor The scaling factor applied to the UR5 parameters.
     * @param exploitRedundancy Flag indicating whether to exploit redundancy (true) or not (false).
     * @param outputFile The output stream to write debug information.
     * @return The joint velocity vector dotQ.
     * 
     * This function calculates the inverse differential kinematic control to generate joint velocities for the UR5 robotic arm.
     * It first computes the Jacobian matrix J for the current arm configuration, then calculates the orientation error and the determinant of J.
     * Depending on the value of exploitRedundancy and the singularity condition, it either computes dotQ directly using the inverse Jacobian,
     * or applies damped pseudo-inverse if near singularity, or returns a singular dotQ vector.
     * 
     * @note This function assumes that the UR5 robotic arm is represented with quaternion orientation.
     * @note The output debug information is written to the specified outputFile stream.
     */
    //FUNZIONE per CALCOLO la DERIVATA dei QUATERNIONI dei joints ------------------------------------------------------------------------
    Eigen::VectorXd invDiffKinematiControlCompleteQuaternion(
        const Eigen::VectorXd& q,           // vettore parametri attuali dei joint   
        const Eigen::VectorXd& xe,          // configurazione end effector corrente
        const Eigen::VectorXd& xd,          // configurazione end effector desiderata
        const Eigen::VectorXd& vd,          // velocità lineare desiderata
        const Eigen::VectorXd& omegad,      // velocità angolare desiderata
        const Eigen::Quaterniond& qe,       // quaternione configurazione end effector corrente 
        const Eigen::Quaterniond& qd,       // quaternione configurazione end effector desiderata
        const Eigen::MatrixXd& Kp,          // matrice di errore lineare     
        const Eigen::MatrixXd& Kq,          // matrice di errore quaternione
        double scaleFactor,
        double exploitRedundancy,
        std::ofstream& outputFile           // file di stampa 
    );


    /**
     * @brief Performs position interpolation between initial and final positions based on normalized time.
     * 
     * This function interpolates between initial and final positions based on the normalized time parameter 't' between 0 (start of motion) 
     * and 1 (end of motion).
     * 
     * @param tb The current time instant.
     * @param Tf The total duration of the motion.
     * @param xe0 The initial position matrix.
     * @param xef The final position matrix.
     * @return The interpolated position matrix at the current time instant.
     * 
     * If the current time 't' exceeds 1, indicating that the current time has surpassed the total duration, the function returns the final
     * position matrix 'xef'. Otherwise, it performs linear interpolation between 'xe0' and 'xef' based on 't'.
     * 
     */
    Eigen::MatrixXd pd(double tb, double Tf, Eigen::MatrixXd xe0, Eigen::MatrixXd xef);


    /**
     * @brief Performs orientation interpolation between initial and final orientations based on normalized time.
     * 
     * This function interpolates between initial and final orientations based on the normalized time parameter 't' between 0 (start of motion) and 1 (end of motion).
     * 
     * @param tb The current time instant.
     * @param Tf The total duration of the motion.
     * @param phief The final orientation matrix.
     * @param phie0 The initial orientation matrix.
     * @return The interpolated orientation matrix at the current time instant.
     * 
     * If the current time 't' exceeds 1, indicating that the current time has surpassed the total duration, the function returns the final 
     * orientation matrix 'phief'. Otherwise, it performs linear interpolation between 'phie0' and 'phief' based on 't'.
     * 
     */
    Eigen::MatrixXd phid(double tb, double Tf, Eigen::MatrixXd phief, Eigen::MatrixXd phie0);


    /**
     * @brief Performs quaternion interpolation between initial and final quaternions based on normalized time.
     * 
     * This function interpolates between initial and final quaternions based on the normalized time parameter 't' between 0 (start of motion) and 1 (end of motion).
     * 
     * @param tb The current time instant.
     * @param Tf The total duration of the motion.
     * @param q0 The initial quaternion.
     * @param qf The final quaternion.
     * @return The interpolated quaternion at the current time instant.
     * 
     * If the current time 't' exceeds 1, indicating that the current time has surpassed the total duration, the function returns the final 
     * quaternion 'qf'. Otherwise, it performs spherical linear interpolation (slerp) between 'q0' and 'qf' based on 't'.
     * 
     */
    Eigen::Quaterniond qd(double tb, double Tf, Eigen::Quaterniond q0, Eigen::Quaterniond qf);


    /**
     * @brief Converts Euler angles to a rotation matrix.
     * 
     * This function converts Euler angles specified in radians to a rotation matrix based on the specified order of rotations.
     * 
     * @param euler_angles The Euler angles in radians (in the order specified by the 'order' parameter).
     * @param order The order of rotations ('XYZ' or 'ZYX').
     * @return The rotation matrix corresponding to the given Euler angles and rotation order.
     * 
     * The function constructs the rotation matrix based on the given Euler angles and rotation order. It supports two commonly used rotation orders:
     * - 'XYZ': Rotation about the X-axis followed by rotation about the Y-axis followed by rotation about the Z-axis.
     * - 'ZYX': Rotation about the Z-axis followed by rotation about the Y-axis followed by rotation about the X-axis.
     * XYZ rotation will always be used in the code, however during the translation phase from Matlab code to C++ we preferred to remain 
     * faithful to the original Matlab function, so as not to have problems for any future implementation
     * 
     * If the provided rotation order is not supported, the function throws an invalid_argument exception.
     * 
     * @note This function assumes that Euler angles are specified in radians.
     */
    Eigen::Matrix3d euler2RotationMatrix(const Eigen::Vector3d& euler_angles, const std::string& order);

    
    /**
     * @brief Inverse Differential Kinematic Control Simulation with Complete Quaternion
     * 
     * This function performs inverse differential kinematic control simulation using complete quaternions.
     * 
     * @param TH0 Initial joint configurations.
     * @param Kp Linear error matrix.
     * @param Kq Quaternion error matrix.
     * @param T Vector containing time steps.
     * @param minT Minimum number of time steps.
     * @param maxT Maximum number of time steps.
     * @param Dt Time step size.
     * @param scaleFactor Scale factor.
     * @param Tf Final time of motion.
     * @param xe0 Initial end-effector position.
     * @param xef Final end-effector position.
     * @param q0 Initial end-effector quaternion.
     * @param qf Final end-effector quaternion.
     * @param exploitRedundancy Flag indicating whether to exploit redundancy.
     * @param outputFile Output file stream to write the results.
     * @return Matrix containing joint configurations for each time step.
     * 
     * This function simulates the inverse differential kinematic control of a robotic manipulator using complete quaternions.
     * It takes the initial joint configurations, linear and quaternion error matrices, time steps, minimum and maximum number of time steps,
     * time step size, scale factor, final time of motion, initial and final end-effector positions and quaternions,
     * and a flag indicating whether to exploit redundancy. It returns a matrix containing joint configurations for each time step.
     * 
     * The function iterates over each time step, computes the end-effector position and orientation at the beginning of the step using forward 
     * kinematics, calculates the desired end-effector position and orientation changes within the current time step, determines the desired 
     * end-effector linear and angular velocities, computes the inverse differential kinematic control to obtain joint velocity changes, 
     * updates the joint configurations for the next time step, and stores the joint configurations in a matrix. If a singularity is encountered 
     * during the computation, a 1x1 matrix containing a zero is returned.
     * 
     * @note The output joint configurations are stored in a matrix, where each row corresponds to a time step and contains the joint 
     * configurations for that step.
     * 
     */
    Eigen::MatrixXd invDiffKinematicControlSimCompleteQuaternion(
        const Eigen::VectorXd& TH0,     // vettore q che contiene le configurazioni iniziale di joint
        const Eigen::MatrixXd& Kp,      // matrice di errore lineare
        const Eigen::MatrixXd& Kq,      // matrice di errore quaternione
        const Eigen::VectorXd& T,       // vettore che contiene i step temporali ( limite iniziale )
        double minT,                    // numero minimo di step temporali 
        double maxT,                    // numero massimo si step temporali 
        double Dt,                      // indicatore il lasso temporale di uno step 
        double scaleFactor,
        double Tf,                      // tempo finale del moto 
        Eigen::MatrixXd xe0,            // posizione iniziale end-effector 
        Eigen::MatrixXd xef,            // posizione finale end-effector
        Eigen::Quaterniond q0,          // quaternione iniziale end-effector 
        Eigen::Quaterniond qf,          // quaternione finale end-effector
        double exploitRedundancy,
        std::ofstream& outputFile       // file di stampa 
    );


    /**
     * @brief Get the first column without NaN values in a matrix
     * 
     * This function returns the first column in the input matrix that does not contain any NaN values.
     * 
     * @param inputMatrix The input matrix
     * @return A NaNColumn struct containing the first column without NaN values and a flag indicating if such a column was found
     * 
     * The function iterates over each column of the input matrix and checks if it contains any NaN values.
     * If a column without NaN values is found, it returns that column and sets all its elements to NaN in the input matrix.
     * If no valid configuration is found, it returns an empty NaNColumn struct with the flag set to false.
     */
    NaNColumn getFirstColumnWithoutNaN(Eigen::MatrixXd& inputMatrix);


    /**
     * @brief Calculate the positions of the joints given the joint angles
     * 
     * This function calculates the positions of the joints of a robot arm given the joint angles.
     * 
     * @param Th The vector of joint angles
     * @param scaleFactor The scale factor for the robot arm dimensions
     * @return A matrix containing the positions of the joints
     */
    Eigen::MatrixXd posizioneGiunti(Eigen::VectorXd Th, double scaleFactor);


    /**
     * @brief Check for collisions between the robot arm and obstacles
     * 
     * This function checks for collisions between the robot arm and obstacles based on the joint positions.
     * 
     * @param Th Matrix containing joint positions
     * @param offset Offset value for collision detection
     * @param dist Distance of the table from the ground
     * @param scaleFactor Scale factor for the robot arm dimensions
     * @param outputFile Output file stream for logging
     * @return True if collisions are detected, false otherwise
     */
    bool checkCollisioni(Eigen::MatrixXd Th, double offset, double dist, double scaleFactor, std::ofstream& outputFile);


    /**
     * @brief Checks for collision or singularity in a trajectory.
     * 
     * This function first checks if the trajectory has a singularity by verifying if it contains only one row. If the trajectory has only one 
     * row, indicating a singularity, it prints a message and returns true. Otherwise, it iterates through each configuration in the trajectory, 
     * obtains the joint positions, and checks for collisions. If any collision is detected, it prints a message and returns true. 
     * If no collision or singularity is found, it returns false.
     * 
     * @param Th The trajectory represented as a matrix where each row represents a configuration.
     * @param scaleFactor The scaling factor applied to the trajectory and joint positions.
     * @param outputFile Reference to the output file stream to log collision or singularity information.
     * 
     * @return A boolean value indicating whether a collision or singularity was detected. I could be true: Collision or singularity detected.
     * Or false: No collision or singularity detected.
     * 
    */
    bool checkCollisionSingularity(Eigen::MatrixXd& Th, double scaleFactor, std::ofstream& outputFile);


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
     * @brief Computes the partial derivative of the determinant of the Jacobian with respect to the i-th element of q.
     * 
     * This function calculates the partial derivative of the determinant of the Jacobian with respect to the i-th element of q.
     * It approximates the derivative using the definition of a derivative and a small increment defined in Kinematic.h.
     * 
     * @param q The vector of joint positions.
     * @param i The index of the element of q with respect to which the derivative is computed.
     * @param scaleFactor The scaling factor applied to the UR5 parameters.
     * @return The partial derivative of the determinant of the Jacobian with respect to the i-th element of q.
     * 
     * This function calculates an approximation of the derivative by incrementing the i-th element of q by a small value h,
     * computing the Jacobians J(q) and J(q + h), and then applying the definition of a derivative.
     * 
     * @note This function assumes that the input vector q represents the joint positions of the UR5 robotic arm.
     * @note The small increment value h is defined in the Kinematic.h header file.
     */
    double DerivataParzialeDetJ(const Eigen::VectorXd& q, int i, double scaleFactor);


    /**
     * @brief Generates a random 3D point with scaled coordinates.
     * 
     * This function generates a random 3D point with coordinates in the range [-0.25, 0.25] for x, [-0.30, 0.18] or [-0.10, 0.10] for y, and 
     * [0.40, 0.65] for z.
     * 
     * @param scaleFactor The scaling factor applied to the generated point.
     * 
     * @return A 3D vector representing the random point with scaled coordinates.
     */
    Eigen::VectorXd randomPoint(double scaleFactor);



    /**
     * @brief Generates an alternative trajectory with two or three steps.
     * 
     * This function generates an alternative trajectory with either two or three steps depending on whether the conditions for a two-step 
     * trajectory are met or not.
     * 
     * For a two-step trajectory: finds a middle point based on the current joint state or generates a random point, calculates the arm 
     * configurations (Th1 and Th2) to reach the middle point and the final point, checks for collision and singularity in each configuration.
     * 
     * For a three-step trajectory: attempts to generate a trajectory by considering three intermediate points, calculates the arm 
     * configurations (Th1, Th2, and Th3) to reach each intermediate point and the final point, checks for collision and singularity in each 
     * configuration.
     * 
     * If a valid trajectory is found, it concatenates the arm configurations into a single matrix and returns it.
     * Otherwise, it returns a matrix with a single entry indicating failure.
     * 
     * @param jointstate The current joint state of the robot.
     * @param Kp The proportional gains matrix for the inverse kinematics controller.
     * @param Kq The derivative gains matrix for the inverse kinematics controller.
     * @param T The joint torque limits.
     * @param minT The minimum torque limit.
     * @param maxT The maximum torque limit.
     * @param DeltaT The time step for trajectory generation.
     * @param scaleFactor The scaling factor applied to the trajectory and joint positions.
     * @param Tf The final time for trajectory execution.
     * @param xe0 The initial end effector position.
     * @param xef The final end effector position.
     * @param q0 The initial orientation quaternion.
     * @param qf The final orientation quaternion.
     * @param outputFile Reference to the output file stream to log trajectory generation information.
     * 
     * @return A matrix containing the generated arm configurations for the alternative trajectory. If a valid trajectory is founf, for a 
     * two-step trajectory: A matrix with 200 rows and 6 columns containing Th1 and Th2. For a three-step trajectory: A matrix with 300 rows 
     * and 6 columns containing Th1, Th2, and Th3.
     * If no valid trajectory is found: A matrix with a single entry indicating failure.
     */
    Eigen::MatrixXd alternativeTrajectory(
        const Eigen::VectorXd& jointstate, 
        const Eigen::MatrixXd& Kp,      
        const Eigen::MatrixXd& Kq,      
        const Eigen::VectorXd& T,       
        double minT,                    
        double maxT,                     
        double DeltaT,                       
        double scaleFactor,
        double Tf,                       
        Eigen::MatrixXd xe0,             
        Eigen::MatrixXd xef,            
        Eigen::Quaterniond q0,           
        Eigen::Quaterniond qf,          
        std::ofstream& outputFile 
    );


    // funzioni per facilitare lo sviluppo; non centrano con il puro calcolo della cinematica
    /**
     * @brief Convert a Eigen::VectorXd to a string
     * 
     * This function converts an Eigen::VectorXd to a string, with each element separated by a space.
     * 
     * @param vec The Eigen::VectorXd to convert
     * @return A string representation of the Eigen::VectorXd
     */
    std::string vectorToString(const Eigen::VectorXd& vec);


    /**
     * @brief Convert an Eigen::Matrix3d to a string
     * 
     * This function converts an Eigen::Matrix3d to a string, with each element separated by a space and each row separated by a newline character.
     * 
     * @param mat The Eigen::Matrix3d to convert
     * @return A string representation of the Eigen::Matrix3d
     */
    std::string matrix3dToString(const Eigen::Matrix3d& mat);


    /**
     * @brief Convert an Eigen::MatrixXd to a string
     * 
     * This function converts an Eigen::MatrixXd to a string, with each element separated by a space and each row separated by a newline character.
     * 
     * @param mat The Eigen::MatrixXd to convert
     * @return A string representation of the Eigen::MatrixXd
     */
    std::string matrixToString(const Eigen::MatrixXd& matrice);


    /**
     * @brief Convert an Eigen::Quaterniond to a string
     * 
     * This function converts an Eigen::Quaterniond to a string representation of its components, separated by commas.
     * 
     * @param quaternion The Eigen::Quaterniond to convert
     * @return A string representation of the Eigen::Quaterniond
     */ 
    std::string quaternioToString(const Eigen::Quaterniond& quaternion); 

    /**
     * @brief Computes the derivative of a function w(q) with respect to each element of q using the chain rule.
     * 
     * This function calculates the derivative of a function w(q) with respect to each element of q,
     * where w(q) is derived from the Jacobian of a UR5 robotic arm with joint positions q.
     * 
     * @param q The vector of joint positions.
     * @param scaleFactor The scaling factor applied to the UR5 parameters.
     * @return The vector of derivatives of w(q) with respect to each element of q.
     * 
     * This function computes the derivative using the chain rule and the determinant of the Jacobian J(q).
     * The derivative of w(q) with respect to each element of q is obtained by multiplying a constant K
     * with the partial derivative of the determinant of J(q) with respect to each element of q.
     * 
     * @note This function assumes that the input vector q represents the joint positions of the UR5 robotic arm.
     * @note The scaling factor is applied to the UR5 parameters before computing the Jacobian.
     */
    Eigen::VectorXd wDerived(const Eigen::VectorXd& q, double scaleFactor);

    

#endif
