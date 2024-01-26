#pragma once
#include <iostream>
#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include <iterator>
#include <functional>

#ifndef __TASKFUNCTION_H__
#define __TASKFUNCTION_H__

    // definisco un tipo BLock per facilitare e rendere la gesstione dei dati più facile ed intuibile
    typedef struct {
        double x[3];
        double phi[3];
        std::string name;
    } Block;

    // Dichiarazione dell'operatore <<
    std::ostream& operator<<(std::ostream& os, const Block& block);


    /* RICHIESTA OBJECT DETECTION
    La funzione richiede al nodo vision planner di eseguire un object detection per trovare tutti i blocci nella scena*/
    std::vector<Block> ask_object_detection(ros::NodeHandle& n);


    /* RICHIESTA MOTION PLAN
    La funzione richiede al nodo motion di eseguire il motion plan per raggiungere la configurazione richiesta: [xef, phief]
    Essa ritorna le configurazione q */
    Eigen::MatrixXd ask_inverse_kinematic(ros::NodeHandle& n, double xef[3], double phief[3]);


    /* RICHIESTA MOVIMENTO BRACCIO
    La funzione richiede a gazebo di muovere il braccio robotico usando la configurazione dei joint q passata*/
    void control_gazebo_arm(ros::NodeHandle& n, std::vector<double> q);


    /* DEFINIZIONE POSIZIONE ORDINATA BLOCCO
    La funzione ritorna la posizione in cui mettere il blocco */
    std::vector<double> define_end_position(std::string block);


    /* FUNZIONI SECONDARIE */
    bool isZero(const std::vector<double>& vettore);
    std::stringstream stampaVector(const std::vector<double>& vec);


#endif
