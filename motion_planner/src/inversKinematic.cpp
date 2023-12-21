#include "ros/ros.h"
#include "motion_planner/InverseKinematic.h"
#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;

struct TransformationMatrices {
    Matrix4d T10, T21, T32, T43, T54, T65, T60;
};

void CinematicaDiretta(const VectorXd& Th, double scaleFactor, Vector3d& pe, Matrix3d& Re, TransformationMatrices& Tm) {
    // Inizializzazione dei vettori A e D e del vettore Alpha (che contiene gli angoli) con i dati del braccio nella posizione di partenza. 
    VectorXd A(6), D(6), Alpha(6);
    A << 0*scaleFactor, -0.425*scaleFactor, -0.3922*scaleFactor, 0, 0, 0;
    D << 0.1625*scaleFactor, 0, 0, 0.1333*scaleFactor, 0.0997*scaleFactor, 0.0996*scaleFactor;
    Alpha << M_PI / 2, 0, 0, M_PI / 2, -M_PI / 2, 0;

    // Questa funzione calcola le matrici di trasformazione
    auto Tij = [](double th, double alpha, double d, double a) -> Matrix4d {
        Matrix4d result;
        result << cos(th), -sin(th) * cos(alpha), sin(th) * sin(alpha), a * cos(th),
                  sin(th), cos(th) * cos(alpha), -cos(th) * sin(alpha), a * sin(th),
                  0, sin(alpha), cos(alpha), d,
                  0, 0, 0, 1;
        return result;
    };

    Tm.T10 = Tij(Th(0), Alpha(0), D(0), A(0));
    Tm.T21 = Tij(Th(1), Alpha(1), D(1), A(1));
    Tm.T32 = Tij(Th(2), Alpha(2), D(2), A(2));
    Tm.T43 = Tij(Th(3), Alpha(3), D(3), A(3));
    Tm.T54 = Tij(Th(4), Alpha(4), D(4), A(4));
    Tm.T65 = Tij(Th(5), Alpha(5), D(5), A(5));

    // Calcola la cinematica totale e restituisce il vettore di traslazione Pe e l'orientamento Re dell'end effector nella posizione finale
    Tm.T60 = Tm.T10 * Tm.T21 * Tm.T32 * Tm.T43 * Tm.T54 * Tm.T65;

    pe = Tm.T60.block<3, 1>(0, 3);
    Re = Tm.T60.block<3, 3>(0, 0);
}

bool inverse(motion_planner::InverseKinematic::Request &req, motion_planner::InverseKinematic::Response &res){
    res.q = req.x + req.y + req.z;
    ROS_INFO("request: x=%f, y=%f, z=%f", req.x, req.y, req.z);
    ROS_INFO("sending back response: [%f]", res.q);
    return true;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "inverse_kinemtic_node");
    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("calculate_inverse_kinematics", inverse);
    ros::spin();
    return 0;
} 