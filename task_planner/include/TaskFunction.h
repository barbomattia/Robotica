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


    /* RICHIESTA MOTION PLAN
    La funzione richiede al nodo motion di eseguire il motion plan per raggiungere la configurazione richiesta: [xef, phief]
    Essa ritorna le configurazione q
    */
    std::vector<double> ask_inverse_kinematic(ros::NodeHandle& n, double xef[3], double phief[3]);

    void control_gazebo_arm(ros::NodeHandle& n, std::vector<double> q);


#endif
