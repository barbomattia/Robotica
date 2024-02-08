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
#define BLOCK_WIDTH_MIN 0.1

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
     * @brief return a random int value between the lower and the higher values
     * 
     * @param min lower value
     * @param max higher value
     * @return int 
     */
    int random(int min, int max);

    /*CINEMATICA DIRETTA --------------------------------------------------------------------------------------------------------- 
    PARAMETRI:
    - Th: vettore dei parametri q dei joints
    - scaleFactor: variabile che consente di adattare le dimensioni del modello del robot UR5 */

    /**
     * @brief Function that calculates the direct kinematics and returns a vector containing the position reached by the end effector and a matrix corresponding to the orientation of the end effector in the world frame
     * 
     * @param Th vector containing the q values ​​of the joints
     * @param scaleFactor scale factor
     * @return CinDir 
     */
    CinDir CinematicaDiretta(const Eigen::VectorXd& Th, double scaleFactor);


    /*CINEMATICA INVERSA --------------------------------------------------------------------------------------------------------- 
    PARAMETRI:
    - p60: posizione end-effector
    - R60: rotazione end-effector
    - scaleFactor: variabile che consente di adattare le dimensioni del modello del robot UR5 */
    Eigen::MatrixXd cinematicaInversa(Eigen::Vector3d p60, Eigen::Matrix3d R60, double scaleFactor);


    /*FUNZIONE CALCOLO JACOBIANA ---------------------------------------------------------------------------------------------------
    PARAMETRI:
    - Th: vettore dei parametri q dei joints
    - scaleFactor: variabile che consente di adattare le dimensioni del modello del robot UR5 */
    Eigen::MatrixXd ur5Jac(const Eigen::VectorXd& Th, double scaleFactor);


    //FUNZIONE per CALCOLO CONIUGATA di un QUATERNIONE -------------------------------------------------------------------------------------
    Eigen::Quaterniond quatconj(const Eigen::Quaterniond& q);
       


    //FUNZIONE per CALCOLO PRODOTTO tra QUATERNIONI --------------------------------------------------------------------------------------
    Eigen::Quaterniond quatmultiply(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2);
      

    //FUNZIONE che RITORNA la PARTE VETTORIALE di un QUATERNION --------------------------------------------------------------------------
    Eigen::Vector3d parts(const Eigen::Quaterniond& q); 
        
    
    // FUNZIONE per CALCOLO della PSEUDOINVERSA DESTRA SMORZATA
    Eigen::MatrixXd dampedPseudoInverse(const Eigen::MatrixXd& J);


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


    /*INTERPOLAZIONE LINEARE tra due POSIZIONI dell'end effector su un tempo NORMALIZZATO ----------------------------------------
    PARAMETRI:
    - xe0: posizione iniziale end-effector
    - xef: posizione finale end-effector
    - tb: tempo attuale 
    - Tf: tempo totale a disposizione per il moto */
    Eigen::MatrixXd pd(double tb, double Tf, Eigen::MatrixXd xe0, Eigen::MatrixXd xef);


    /*INTERPOLAZIONE LINEARE tra due ORIENTAZIONI dell'end effector su un tempo NORMALIZZATO ----------------------------------------
    PARAMETRI:
    - phie0: orientazione (angoli Eulero) iniziali end-effector
    - phief: orientazione (angoli Eulero) finali end-effector
    - tb: tempo attuale 
    - Tf: tempo totale a disposizione per il moto */
    Eigen::MatrixXd phid(double tb, double Tf, Eigen::MatrixXd phief, Eigen::MatrixXd phie0);


    /*INTERPOLAZIONE SFERICA tra due CONFIGURAZIONE dell'end effector su un tempo NORMALIZZATO ----------------------------------------
    PARAMETRI:
    - q0: quaternione della configurazione iniziali end-effector
    - phief: quaternione della configurazione finali end-effector
    - tb: tempo attuale 
    - Tf: tempo totale a disposizione per il moto  */
    Eigen::Quaterniond qd(double tb, double Tf, Eigen::Quaterniond q0, Eigen::Quaterniond qf);


    //FUNZIONE per CONVERTIRE una ORIENTAZIONE (angoli EULERO) in una MATRICE di ROTAZIONE ----------------------------------------------
    Eigen::Matrix3d euler2RotationMatrix(const Eigen::Vector3d& euler_angles, const std::string& order);

    // FUNZIONE per CALCOLO CONFIGURAZIONI JOINT usando un Control basato sui QUATERNIONI e su INTERPOLAZIONE SFERICA --------------------
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


    NaNColumn getFirstColumnWithoutNaN(Eigen::MatrixXd& inputMatrix);

    Eigen::MatrixXd posizioneGiunti(Eigen::VectorXd Th, double scaleFactor);

    bool checkCollisioni(Eigen::MatrixXd Th, double offset, double dist, double scaleFactor, std::ofstream& outputFile);
    bool checkCollisionSingularity(Eigen::MatrixXd& Th, double scaleFactor, std::ofstream& outputFile);

    Eigen::Quaterniond slerpFunction(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2, double t);

    double Gripper(std::string blockName);

    double DerivataParzialeDetJ(const Eigen::VectorXd& q, int i, double scaleFactor);

    //Eigen::VectorXd wDerived(const Eigen::VectorXd& q, double scaleFactor);

    Eigen::VectorXd randomPoint(double scaleFactor);

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
    std::string vectorToString(const Eigen::VectorXd& vec);
    std::string matrix3dToString(const Eigen::Matrix3d& mat);
    std::string matrixToString(const Eigen::MatrixXd& matrice); 
    std::string quaternioToString(const Eigen::Quaterniond& quaternion); 

    

#endif
