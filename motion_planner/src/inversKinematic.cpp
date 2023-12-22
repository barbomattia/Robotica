#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense> 
#include "ros/ros.h"
#include "motion_planner/InverseKinematic.h"
#include <iostream>


struct TransformationMatrices {
    Eigen::Matrix4d T10, T21, T32, T43, T54, T65, T60;
};


//CINEMATICA DIRETTA
void CinematicaDiretta(const Eigen::VectorXd& Th, double scaleFactor, Eigen::Vector3d& pe, Eigen::Matrix3d& Re, TransformationMatrices& Tm) {
    // Inizializzazione dei vettori A e D e del vettore Alpha (che contiene gli angoli) con i dati del braccio nella posizione di partenza. 
    Eigen::VectorXd A(6), D(6), Alpha(6);
    A << 0 * scaleFactor, -0.425 * scaleFactor, -0.3922 * scaleFactor, 0, 0, 0;
    D << 0.1625 * scaleFactor, 0, 0, 0.1333 * scaleFactor, 0.0997 * scaleFactor, 0.0996 * scaleFactor;
    Alpha << M_PI / 2, 0, 0, M_PI / 2, -M_PI / 2, 0;

    // Questa funzione calcola le matrici di trasformazione
    auto Tij = [](double th, double alpha, double d, double a) -> Eigen::Matrix4d {
        Eigen::Matrix4d result;
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


 Eigen::MatrixXd cinematicaInversa(Eigen::Vector3d p60, Eigen::Matrix3d R60, double scaleFactor) {
    // Inizializzazione vettori
    Eigen::VectorXd A(6);
    A << 0, -0.425, -0.3922, 0, 0, 0;
    A *= scaleFactor;

    Eigen::VectorXd D(6);
    D << 0.1625, 0, 0, 0.1333, 0.0997, 0.0996;
    D *= scaleFactor;

    Eigen::VectorXd Alpha(6);
    Alpha << M_PI / 2, 0, 0, M_PI / 2, -M_PI / 2, 0;

    // Matrice di trasformazione
    auto Tij = [](double th, double alpha, double d, double a) -> Eigen::Matrix4d {
        Eigen::Matrix4d result;
        result << cos(th), -sin(th) * cos(alpha), sin(th) * sin(alpha), a * cos(th),
                  sin(th), cos(th) * cos(alpha), -cos(th) * sin(alpha), a * sin(th),
                  0, sin(alpha), cos(alpha), d,
                  0, 0, 0, 1;
        return result;
    };

    auto almzero = [](double x) { return std::abs(x) < 1e-7; };


    Eigen::Matrix4d T60;
    T60.block<3, 3>(0, 0) = R60;
    T60.col(3).head(3) = p60;
    T60.row(3) << 0, 0, 0, 1;

    // Calcolo th1
    Eigen::Vector4d p50 = T60 * Eigen::Vector4d(0, 0, -D(5), 1);
    double psi = atan2(p50(1), p50(0));
    double p50xy = hypot(p50(1), p50(0));
    
    if (p50xy < D(3)) {
        std::cerr << "Posizione non raggiungibile" << std::endl;
        return Eigen::VectorXd::Constant(6, NAN);
    }

    // Calcolo phi1_1 e phi1_2
    double phi1_1 = acos(D(3) / p50xy);
    double phi1_2 = -phi1_1;  

    // Calcolo th1_1 e th1_2
    double th1_1 = psi + phi1_1 + M_PI / 2;
    double th1_2 = psi + phi1_2 + M_PI / 2;

    // Calcolo p61z_1 e p61z_2
    double p61z_1 = p60(0) * sin(th1_1) - p60(1) * cos(th1_1);
    double p61z_2 = p60(0) * sin(th1_2) - p60(1) * cos(th1_2);

    // Calcolo th5_1_1, th5_1_2, th5_2_1, th5_2_2
    double th5_1_1 = acos((p61z_1 - D(3)) / D(5));
    double th5_1_2 = -acos((p61z_1 - D(3)) / D(5));
    double th5_2_1 = acos((p61z_2 - D(3)) / D(5));
    double th5_2_2 = -acos((p61z_2 - D(3)) / D(5));

    // Caloclo T10_1 e T10_2
    Eigen::Matrix4d T10_1 = Tij(th1_1, Alpha(0), D(0), A(0));
    Eigen::Matrix4d T10_2 = Tij(th1_2, Alpha(0), D(0), A(0));

    // Calcolo T16_1 e T16_2
    Eigen::Matrix4d T16_1 = (T10_1.inverse() * T60).inverse();
    Eigen::Matrix4d T16_2 = (T10_2.inverse() * T60).inverse();

    // Calcolo zy_1, zx_1, zy_2, zx_2
    double zy_1 = T16_1(1, 2);
    double zx_1 = T16_1(0, 2);

    double zy_2 = T16_2(1, 2);
    double zx_2 = T16_2(0, 2);

    double th6_1_1;
    if (almzero(sin(th5_1_1)) || (almzero(zy_1) && almzero(zx_1))) {
        std::cerr << "singola configurazione" << std::endl;
        th6_1_1 = 0;
    } else {
        th6_1_1 = atan2((-zy_1 / sin(th5_1_1)), (zx_1 / sin(th5_1_1)));
    }

    // Calcolo th6_1_2
    double th6_1_2;
    if (almzero(sin(th5_1_2)) || (almzero(zy_1) && almzero(zx_1))) {
        std::cerr << "singola configurazione" << std::endl;
        th6_1_2 = 0;
    } else {
        th6_1_2 = atan2((-zy_1 / sin(th5_1_2)), (zx_1 / sin(th5_1_2)));
    }

    // Calcolo th6_2_1
    double th6_2_1;
    if (almzero(sin(th5_2_1)) || (almzero(zy_2) && almzero(zx_2))) {
        std::cerr << "singola configurazione" << std::endl;
        th6_2_1 = 0;
    } else {
        th6_2_1 = atan2((-zy_2 / sin(th5_2_1)), (zx_2 / sin(th5_2_1)));
    }

    // Calcolo th6_2_2
    double th6_2_2;
    if (almzero(sin(th5_2_2)) || (almzero(zy_2) && almzero(zx_2))) {
        std::cerr << "singola configurazione" << std::endl;
        th6_2_2 = 0;
    } else {
        th6_2_2 = atan2((-zy_2 / sin(th5_2_2)), (zx_2 / sin(th5_2_2)));
    }

    Eigen::Matrix4d T61_1 = T16_1.inverse();
    Eigen::Matrix4d T61_2 = T16_2.inverse();

    // Calcolo T54_1_1, T54_1_2, T54_2_1, T54_2_2
    Eigen::Matrix4d T54_1_1 = Tij(th5_1_1, Alpha(4), D(4), A(4));
    Eigen::Matrix4d T54_1_2 = Tij(th5_1_2, Alpha(4), D(4), A(4));
    Eigen::Matrix4d T54_2_1 = Tij(th5_2_1, Alpha(4), D(4), A(4));
    Eigen::Matrix4d T54_2_2 = Tij(th5_2_2, Alpha(4), D(4), A(4));

    // Calcolo T65_1_1, T65_1_2, T65_2_1, T65_2_2
    Eigen::Matrix4d T65_1_1 = Tij(th6_1_1, Alpha(5), D(5), A(5));
    Eigen::Matrix4d T65_1_2 = Tij(th6_1_2, Alpha(5), D(5), A(5));
    Eigen::Matrix4d T65_2_1 = Tij(th6_2_1, Alpha(5), D(5), A(5));
    Eigen::Matrix4d T65_2_2 = Tij(th6_2_2, Alpha(5), D(5), A(5));

    // Calcolo T41_1_1, T41_1_2, T41_2_1, T41_2_2
    Eigen::Matrix4d T41_1_1 = T61_1 * (T54_1_1 * T65_1_1).inverse();
    Eigen::Matrix4d T41_1_2 = T61_1 * (T54_1_2 * T65_1_2).inverse();
    Eigen::Matrix4d T41_2_1 = T61_2 * (T54_2_1 * T65_2_1).inverse();
    Eigen::Matrix4d T41_2_2 = T61_2 * (T54_2_2 * T65_2_2).inverse();

    // Calcolo P31_1_1, P31_1_2, P31_2_1, P31_2_2
    Eigen::Vector4d inputVector(0, -D[3], 0, 1);

    Eigen::Vector3d P31_1_1 = (T41_1_1 * inputVector).head<3>();
    Eigen::Vector3d P31_1_2 = (T41_1_2 * inputVector).head<3>();
    Eigen::Vector3d P31_2_1 = (T41_2_1 * inputVector).head<3>();
    Eigen::Vector3d P31_2_2 = (T41_2_2 * inputVector).head<3>();

    // Calcolo th3_1_1_1 e th3_1_1_2
    double C = (P31_1_1.squaredNorm() - (A(1) * A(1)) - (A(2) * A(2))) / (2 * A(1) * A(2));
    double th3_1_1_1, th3_1_1_2;

    if (std::abs(C) > 1) {
        std::cerr << "Punto fuori dal workspace" << std::endl;
        th3_1_1_1 = th3_1_1_2 = NAN;
    } else {
        th3_1_1_1 = acos(C);
        th3_1_1_2 = -acos(C);
    }

    // Calcolo th3_1_2_1 e th3_1_2_2
    C = (P31_1_2.squaredNorm() - A(1) * A(1) - A(2) * A(2)) / (2 * A(1) * A(2));
    double th3_1_2_1, th3_1_2_2;

    if (std::abs(C) > 1) {
        std::cerr << "Punto fuori dal workspace" << std::endl;
        th3_1_2_1 = th3_1_2_2 = NAN;
    } else {
        th3_1_2_1 = acos(C);
        th3_1_2_2 = -acos(C);
    }

    // Calcolo th3_2_1_1 e th3_2_1_2
    C = (P31_2_1.squaredNorm() - A(1) * A(1) - A(2) * A(2)) / (2 * A(1) * A(2));
    double th3_2_1_1, th3_2_1_2;

    if (std::abs(C) > 1) {
        std::cerr << "Punto fuori dal workspace" << std::endl;
        th3_2_1_1 = th3_2_1_2 = NAN;
    } else {
        th3_2_1_1 = acos(C);
        th3_2_1_2 = -acos(C);
    }

    // Calcolo th3_2_2_1 and th3_2_2_2
    C = (P31_2_2.squaredNorm() - A(1) * A(1) - A(2) * A(2)) / (2 * A(1) * A(2));
    double th3_2_2_1, th3_2_2_2;

    if (std::abs(C) > 1) {
        std::cerr << "Punto fuori dal workspace" << std::endl;
        th3_2_2_1 = th3_2_2_2 = NAN;
    } else {
        th3_2_2_1 = acos(C);
        th3_2_2_2 = -acos(C);
    }

    // Calcolo th2_1_1_1 and th2_1_1_2
    double th2_1_1_1 = -atan2(P31_1_1(1), -P31_1_1(0)) + asin((A(2) * sin(th3_1_1_1)) / P31_1_1.norm());
    double th2_1_1_2 = -atan2(P31_1_1(1), -P31_1_1(0)) + asin((A(2) * sin(th3_1_1_2)) / P31_1_1.norm());

    // Calcolo th2_1_2_1 and th2_1_2_2
    double th2_1_2_1 = -atan2(P31_1_2(1), -P31_1_2(0)) + asin((A(2) * sin(th3_1_2_1)) / P31_1_2.norm());
    double th2_1_2_2 = -atan2(P31_1_2(1), -P31_1_2(0)) + asin((A(2) * sin(th3_1_2_2)) / P31_1_2.norm());

    // Calcolo th2_2_1_1 and th2_2_1_2
    double th2_2_1_1 = -atan2(P31_2_1(1), -P31_2_1(0)) + asin((A(2) * sin(th3_2_1_1)) / P31_2_1.norm());
    double th2_2_1_2 = -atan2(P31_2_1(1), -P31_2_1(0)) + asin((A(2) * sin(th3_2_1_2)) / P31_2_1.norm());

    // Calcolo th2_2_2_1 and th2_2_2_2
    double th2_2_2_1 = -atan2(P31_2_2(1), -P31_2_2(0)) + asin((A(2) * sin(th3_2_2_1)) / P31_2_2.norm());
    double th2_2_2_2 = -atan2(P31_2_2(1), -P31_2_2(0)) + asin((A(2) * sin(th3_2_2_2)) / P31_2_2.norm());

    // Calcolo th4_1_1_1
    Eigen::Matrix4d T21 = Tij(th2_1_1_1, Alpha(1), D(1), A(1));
    Eigen::Matrix4d T32 = Tij(th3_1_1_1, Alpha(2), D(2), A(2));
    Eigen::Matrix4d T41 = T41_1_1;
    Eigen::Matrix4d T43 = (T21 * T32).inverse() * T41;
    
    double xy = T43(1, 0);
    double xx = T43(0, 0);
    double th4_1_1_1 = atan2(xy, xx);

    // Calcolo th4_1_1_2
    T21 = Tij(th2_1_1_2, Alpha(1), D(1), A(1));
    T32 = Tij(th3_1_1_2, Alpha(2), D(2), A(2));
    T41 = T41_1_1;
    T43 = (T21 * T32).inverse() * T41;

    xy = T43(1, 0);
    xx = T43(0, 0);
    double th4_1_1_2 = atan2(xy, xx);

    // Calcolo th4_1_2_1
    T21 = Tij(th2_1_2_1, Alpha(1), D(1), A(1));
    T32 = Tij(th3_1_2_1, Alpha(2), D(2), A(2));
    T41 = T41_1_2;
    T43 = (T21 * T32).inverse() * T41;

    xy = T43(1, 0);
    xx = T43(0, 0);
    double th4_1_2_1 = atan2(xy, xx);

    // Calcolo th4_1_2_2
    T21 = Tij(th2_1_2_2, Alpha(1), D(1), A(1));
    T32 = Tij(th3_1_2_2, Alpha(2), D(2), A(2));
    T41 = T41_1_2;
    T43 = (T21 * T32).inverse() * T41;

    xy = T43(1, 0);
    xx = T43(0, 0);
    double th4_1_2_2 = atan2(xy, xx);

    // Calcolo th4_2_1_1
    T21 = Tij(th2_2_1_1, Alpha(1), D(1), A(1));
    T32 = Tij(th3_2_1_1, Alpha(2), D(2), A(2));
    T41 = T41_2_1;
    T43 = (T21 * T32).inverse() * T41;

    xy = T43(1, 0);
    xx = T43(0, 0);
    double th4_2_1_1 = atan2(xy, xx);

    // Calcolo th4_2_1_2
    T21 = Tij(th2_2_1_2, Alpha(1), D(1), A(1));
    T32 = Tij(th3_2_1_2, Alpha(2), D(2), A(2));
    T41 = T41_2_1;
    T43 = (T21 * T32).inverse() * T41;

    xy = T43(1, 0);
    xx = T43(0, 0);
    double th4_2_1_2 = atan2(xy, xx);

    // Calcolo th4_2_2_1
    T21 = Tij(th2_2_2_1, Alpha(1), D(1), A(1));
    T32 = Tij(th3_2_2_1, Alpha(2), D(2), A(2));
    T41 = T41_2_2;
    T43 = (T21 * T32).inverse() * T41;

    xy = T43(1, 0);
    xx = T43(0, 0);
    double th4_2_2_1 = atan2(xy, xx);

    // Calcolo th4_2_2_2
    T21 = Tij(th2_2_2_2, Alpha(1), D(1), A(1));
    T32 = Tij(th3_2_2_2, Alpha(2), D(2), A(2));
    T41 = T41_2_2;
    T43 = (T21 * T32).inverse() * T41;

    xy = T43(1, 0);
    xx = T43(0, 0);
    double th4_2_2_2 = atan2(xy, xx);

    // Calcolo matrice delle configurazioni
    Eigen::MatrixXd Th(6, 8); 

    Th << th1_1, th1_1, th1_1, th1_1, th1_2, th1_2, th1_2, th1_2,
        th2_1_1_1, th2_1_1_2, th2_1_2_1, th2_1_2_2, th2_2_2_1, th2_2_1_2, th2_2_2_1, th2_2_2_2,
        th3_1_1_1, th3_1_1_2, th3_1_2_1, th3_1_2_2, th3_2_2_1, th3_2_1_2, th3_2_2_1, th3_2_2_2,
        th4_1_1_1, th4_1_1_2, th4_1_2_1, th4_1_2_2, th4_2_2_1, th4_2_1_2, th4_2_2_1, th4_2_2_2,
        th5_1_1, th5_1_1, th5_1_2, th5_1_2, th5_2_2, th5_2_1, th5_2_2, th5_2_2,
        th6_1_1, th6_1_1, th6_1_2, th6_1_2, th6_2_2, th6_2_1, th6_2_2, th6_2_2;

    return Th;
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