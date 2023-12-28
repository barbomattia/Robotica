#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <functional>


struct TransformationMatrices {
    Eigen::Matrix4d T10, T21, T32, T43, T54, T65, T60;
};

struct CinDir{
    Eigen::Vector3d pe;
    Eigen::Matrix3d Re;
};


//CINEMATICA DIRETTA
CinDir CinematicaDiretta(const Eigen::VectorXd& Th, double scaleFactor) {
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

    TransformationMatrices Tm;   
    Tm.T10 = Tij(Th(0), Alpha(0), D(0), A(0));
    Tm.T21 = Tij(Th(1), Alpha(1), D(1), A(1));
    Tm.T32 = Tij(Th(2), Alpha(2), D(2), A(2));
    Tm.T43 = Tij(Th(3), Alpha(3), D(3), A(3));
    Tm.T54 = Tij(Th(4), Alpha(4), D(4), A(4));
    Tm.T65 = Tij(Th(5), Alpha(5), D(5), A(5));

    // Calcola la cinematica totale e restituisce il vettore di traslazione Pe e l'orientamento Re dell'end effector nella posizione finale
    Tm.T60 = Tm.T10 * Tm.T21 * Tm.T32 * Tm.T43 * Tm.T54 * Tm.T65;

    Eigen::Vector3d pe = Tm.T60.block<3, 1>(0, 3);
    Eigen::Matrix3d Re = Tm.T60.block<3, 3>(0, 0);

    CinDir result;
    result.pe = pe;
    result.Re = Re;
    return result;
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

Eigen::MatrixXd ur5Jac(const Eigen::VectorXd& Th, double scaleFactor) {
    Eigen::VectorXd A(6);
    A << 0, -0.425*scaleFactor, -0.3922*scaleFactor, 0, 0, 0;
    
    Eigen::VectorXd D(6);
    D << 0.1625*scaleFactor, 0, 0, 0.1333*scaleFactor, 0.0997*scaleFactor, 0.0996*scaleFactor;
    
    double A1 = A(0), A2 = A(1), A3 = A(2), A4 = A(3), A5 = A(4), A6 = A(5);
    double D1 = D(0), D2 = D(1), D3 = D(2), D4 = D(3), D5 = D(4), D6 = D(5);
    
    double th1 = Th(0);
    double th2 = Th(1);
    double th3 = Th(2);
    double th4 = Th(3);
    double th5 = Th(4);
    double th6 = Th(5);
    
    Eigen::MatrixXd J(6, 6);
    
    // J1
    J.col(0) << D5 * (cos(th1) * cos(th5) + cos(th2 + th3 + th4) * sin(th1) * sin(th5)) + D4 * cos(th1) - A2 * cos(th2) * sin(th1) - D5 * sin(th2 + th3 + th4) * sin(th1) - A3 * cos(th2) * cos(th3) * sin(th1) + A3 * sin(th1) * sin(th2) * sin(th3),
                D5 * (cos(th5) * sin(th1) - cos(th2 + th3 + th4) * cos(th1) * sin(th5)) + D4 * sin(th1) + A2 * cos(th1) * cos(th2) + D5 * sin(th2 + th3 + th4) * cos(th1) + A3 * cos(th1) * cos(th2) * cos(th3) - A3 * cos(th1) * sin(th2) * sin(th3),
                0,
                0,
                0,
                1;
    
    // J2
    J.col(1) << -cos(th1) * (A3 * sin(th2 + th3) + A2 * sin(th2) + D5 * (sin(th2 + th3) * sin(th4) - cos(th2 + th3) * cos(th4)) - D5 * sin(th5) * (cos(th2 + th3) * sin(th4) + sin(th2 + th3) * cos(th4))),
                -sin(th1) * (A3 * sin(th2 + th3) + A2 * sin(th2) + D5 * (sin(th2 + th3) * sin(th4) - cos(th2 + th3) * cos(th4)) - D5 * sin(th5) * (cos(th2 + th3) * sin(th4) + sin(th2 + th3) * cos(th4))),
                A3 * cos(th2 + th3) - (D5 * sin(th2 + th3 + th4 + th5)) / 2 + A2 * cos(th2) + (D5 * sin(th2 + th3 + th4 - th5)) / 2 + D5 * sin(th2 + th3 + th4),
                sin(th1),
                -cos(th1),
                0;
    
    // J3
    J.col(2) << cos(th1) * (D5 * cos(th2 + th3 + th4) - A3 * sin(th2 + th3) + D5 * sin(th2 + th3 + th4) * sin(th5)),
                sin(th1) * (D5 * cos(th2 + th3 + th4) - A3 * sin(th2 + th3) + D5 * sin(th2 + th3 + th4) * sin(th5)),
                A3 * cos(th2 + th3) - (D5 * sin(th2 + th3 + th4 + th5)) / 2 + (D5 * sin(th2 + th3 + th4 - th5)) / 2 + D5 * sin(th2 + th3 + th4),
                sin(th1),
                -cos(th1),
                0;
    
    // J4
    J.col(3) << D5 * cos(th1) * (cos(th2 + th3 + th4) + sin(th2 + th3 + th4) * sin(th5)),
                D5 * sin(th1) * (cos(th2 + th3 + th4) + sin(th2 + th3 + th4) * sin(th5)),
                D5 * (sin(th2 + th3 + th4 - th5) / 2 + sin(th2 + th3 + th4) - sin(th2 + th3 + th4 + th5) / 2),
                sin(th1),
                -cos(th1),
                0;
    
    // J5
    J.col(4) << D5 * cos(th1) * cos(th2) * cos(th5) * sin(th3) * sin(th4) - D5 * cos(th1) * cos(th2) * cos(th3) * cos(th4) * cos(th5) - D5 * sin(th1) * sin(th5) + D5 * cos(th1) * cos(th3) * cos(th5) * sin(th2) * sin(th4) + D5 * cos(th1) * cos(th4) * cos(th5) * sin(th2) * sin(th3),
                D5 * cos(th1) * sin(th5) + D5 * cos(th2) * cos(th5) * sin(th1) * sin(th3) * sin(th4) + D5 * cos(th3) * cos(th5) * sin(th1) * sin(th2) * sin(th4) + D5 * cos(th4) * cos(th5) * sin(th1) * sin(th2) * sin(th3) - D5 * cos(th2) * cos(th3) * cos(th4) * cos(th5) * sin(th1),
                -D5 * (sin(th2 + th3 + th4 - th5) / 2 + sin(th2 + th3 + th4 + th5) / 2),
                sin(th2 + th3 + th4) * cos(th1),
                sin(th2 + th3 + th4) * sin(th1),
                -cos(th2 + th3 + th4);
    
    // J6
    J.col(5) << 0,
                0,
                0,
                cos(th5) * sin(th1) - cos(th2 + th3 + th4) * cos(th1) * sin(th5),
                -cos(th1) * cos(th5) - cos(th2 + th3 + th4) * sin(th1) * sin(th5),
                -sin(th2 + th3 + th4) * sin(th5);

    return J;
}

// Funzione per calcolare la coniugata di un quaternione
Eigen::Quaterniond quatconj(const Eigen::Quaterniond& q) {
    return Eigen::Quaterniond(q.w(), -q.x(), -q.y(), -q.z());
}

Eigen::Quaterniond quatmultiply(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2) {
    return q1 * q2;
}

Eigen::Vector3d parts(const Eigen::Quaterniond& q) {
    return q.vec();
}

// Funzione per calcolare la velocità di giunto controllata per il movimento completo
Eigen::VectorXd invDiffKinematiControlCompleteQuaternion(
    const Eigen::VectorXd& q,           //posizione joint   
    const Eigen::VectorXd& xe,          //posizione end effector corrente
    const Eigen::VectorXd& xd,          //posizione end effector desiderata
    const Eigen::VectorXd& vd,          //velocità desiderata
    const Eigen::VectorXd& omegad,      //velocità angolare desiderata
    const Eigen::Quaterniond& qe,       //quaternione corrente
    const Eigen::Quaterniond& qd,       //cogniugato quaternione
    const Eigen::MatrixXd& Kp,          //matrice di errore lineare     
    const Eigen::MatrixXd& Kq,          //matrice di errore quaternione
    double scaleFactor
) {
    Eigen::MatrixXd J = ur5Jac(q, scaleFactor);

    if (std::abs(J.determinant()) < 1e-3) {
        std::cerr << "vicino ad una singolarità" << std::endl;
        //implementazione
    }
    

     // Quaternion operations
    Eigen::Quaterniond qp = qd * qe.conjugate();
    Eigen::Vector3d eo = qp.vec();

    // Calculate dotQ
    Eigen::VectorXd dotQ = J.inverse() * (Eigen::VectorXd(6) << (vd + Kp * (xd - xe)), (omegad + Kq * eo)).finished();

    return dotQ;
}


// Implementazioni delle funzioni
Eigen::MatrixXd pd(double tb, double Tf, Eigen::MatrixXd xe0, Eigen::MatrixXd xef) {
    double t = tb / Tf;
    Eigen::MatrixXd result(3, 1);
    if (t > 1) {
        result = xef;
    } else {
        result = t * xef + (1 - t) * xe0;
    }
    return result;
}

Eigen::MatrixXd phid(double tb, double Tf, Eigen::MatrixXd phief, Eigen::MatrixXd phie0) {
    double t = tb / Tf;
    Eigen::MatrixXd result(3, 1);
    if (t > 1) {
        result = phief;
    } else {
        result = t * phief + (1 - t) * phie0;
    }
    return result;
}

Eigen::Quaterniond qd(double tb, double Tf, Eigen::Quaterniond q0, Eigen::Quaterniond qf) {
    double t = tb / Tf;
    Eigen::Quaterniond result;
    if (t > 1) {
        result = qf;
    } else {
        result = q0.slerp(t, qf);
    }
    return result;
}

Eigen::Matrix3d euler2RotationMatrix(const Eigen::Vector3d& euler_angles, const std::string& order) {
    Eigen::Matrix3d rotation_matrix;

    if (order == "XYZ") {
        rotation_matrix = Eigen::AngleAxisd(euler_angles[0], Eigen::Vector3d::UnitX())
                        * Eigen::AngleAxisd(euler_angles[1], Eigen::Vector3d::UnitY())
                        * Eigen::AngleAxisd(euler_angles[2], Eigen::Vector3d::UnitZ());
    } else if (order == "ZYX") {
        rotation_matrix = Eigen::AngleAxisd(euler_angles[2], Eigen::Vector3d::UnitZ())
                        * Eigen::AngleAxisd(euler_angles[1], Eigen::Vector3d::UnitY())
                        * Eigen::AngleAxisd(euler_angles[0], Eigen::Vector3d::UnitX());
    } else {
        // Add support for other rotation orders if needed
        throw std::invalid_argument("Unsupported rotation order");
    }

    return rotation_matrix;
}

// Funzione di simulazione completa
Eigen::MatrixXd invDiffKinematicControlSimCompleteQuaternion(
    const Eigen::VectorXd& TH0,
    const Eigen::MatrixXd& Kp,
    const Eigen::MatrixXd& Kq,
    const Eigen::VectorXd& T,
    double minT,
    double maxT,
    double Dt,
    double scaleFactor,
    double Tf, 
    Eigen::MatrixXd xe0, Eigen::MatrixXd xef,
    Eigen::Quaterniond q0, Eigen::Quaterniond qf
) {
    std::vector<Eigen::VectorXd> q;

    int L = T.size();
    Eigen::VectorXd qk = TH0;
    q.push_back(qk);

    for (int i = 1; i < L - 1; ++i) {
        auto result = CinematicaDiretta(qk, scaleFactor);
        Eigen::VectorXd xe = result.pe;
        Eigen::Matrix3d Re = result.Re;
        Eigen::Quaterniond qe(Re);

        Eigen::MatrixXd vd1 = (pd(T[i], Tf, xe0, xef) - pd(0, Tf, xe0, xef));
        Eigen::MatrixXd vd = vd1 / Dt;        

        Eigen::Quaterniond qd_t = qd(T[i], Tf, q0, qf);
        Eigen::Quaterniond qd_t_plus_Dt = qd(T[i] + Dt, Tf, q0, qf);

        Eigen::Quaterniond work = quatmultiply(qd_t_plus_Dt, qd_t.conjugate());
        work.coeffs() *= 2.0 / Dt;

        Eigen::Vector3d omegad = parts(work);

        Eigen::VectorXd dotqk = invDiffKinematiControlCompleteQuaternion(
            qk, xe, pd(T[i],Tf, xe0, xef), vd, omegad, qe, qd(T[i],Tf, q0, qf), Kp, Kq, scaleFactor
        );
        qk = qk + dotqk * Dt;

        q.push_back(qk);
    }

    
    // Convert the vector of vectors to a matrix
    Eigen::MatrixXd resultMatrix(q.size(), q[0].size());
    for (size_t i = 0; i < q.size(); ++i) {
        resultMatrix.row(i) = q[i];
    }

    return resultMatrix;
}

Eigen::VectorXd getFirstColumnWithoutNaN(const Eigen::MatrixXd& inputMatrix) {
    Eigen::VectorXd firstColumnWithoutNaN;

    for (int col = 0; col < inputMatrix.cols(); ++col) {
        // Verifica se ci sono valori NaN nella colonna corrente
        if (!inputMatrix.col(col).array().isNaN().any()) {
            // Estrai la prima colonna senza NaN
            firstColumnWithoutNaN = inputMatrix.col(col);
            break;
        }
    }

    return firstColumnWithoutNaN;
}

int main() {

    /*

    //CINEMATICA DIRETTA
    
    Eigen::VectorXd Th(6);
    Th << 0.1,0.2,0.3,0.4,0.5,0.6;
    double scaleFactor = 10.0;

    Eigen::Vector3d pe;
    Eigen::Matrix3d Re;
    TransformationMatrices Tm;
    std::cout << "configurazione iniziale: " << Th.transpose() << std::endl << std::endl;

    std::cout << "CINEMATICA DIRETTA" << std::endl;
    CinematicaDiretta(Th, scaleFactor, pe, Re, Tm);

    std::cout << "vettore (pe): " << pe.transpose() << std::endl;
    std::cout << "matrice orientamento (Re): " << std::endl << Re << std::endl;

    std::cout << std::endl;
    
    //CINEMATICA INVERSA
    std::cout << "CINEMATICA INVERSA" << std::endl;
    //Vector3d p60(3.0, 3.0, 3.0);  
    //Matrix3d R60 = Eigen::Matrix3d::Identity();   
    Eigen::MatrixXd M = cinematicaInversa(pe, Re, scaleFactor);
    
    std::cout << "Matrice delle variabili congiunte M:\n" << M << std::endl;
    
    */

   // Inizializzazione delle variabili globali
    Eigen::MatrixXd xe0(3, 1);
    Eigen::MatrixXd xef(3, 1);
    Eigen::MatrixXd phie0(3, 1);
    Eigen::MatrixXd phief(3, 1);
    double scaleFactor = 10.0;
    double Tf = 10.0;
    double DeltaT = 0.1;
    Eigen::VectorXd T;
    T = Eigen::VectorXd::LinSpaced(static_cast<int>((Tf / DeltaT) + 1), 0, Tf);
        
    xe0 << 0.3, 0.3, 0.1;
    xe0 *= scaleFactor;
    phie0 << 0, 0, 0;
    xef << 0.5, 0.5, 0.5;
    xef *= scaleFactor;
    phief << M_PI / 3, M_PI / 2 - 0.1, M_PI / 3;
    
    Eigen::Matrix3d workM;
    workM = euler2RotationMatrix(phie0, "XYZ");
    Eigen::Quaterniond q0(workM);

    Eigen::Matrix4d Tt0 = Eigen::Matrix4d::Identity();
    Tt0.block<3, 3>(0, 0) = euler2RotationMatrix(phief, "XYZ");
    Tt0.block<3, 1>(0, 3) = xef;

    Eigen::Quaterniond qf(Tt0.block<3, 3>(0, 0));

    // Chiamata alle funzioni
    Eigen::MatrixXd TH0 = cinematicaInversa(pd(0, Tf, xe0, xef), euler2RotationMatrix(phid(0, Tf, phief, phie0), "XYZ"), scaleFactor);
    Eigen::VectorXd M = getFirstColumnWithoutNaN(TH0);
    Eigen::Matrix3d Kp = 10.0 * Eigen::Matrix3d::Identity();
    Eigen::Matrix3d Kq = -10.0 * Eigen::Matrix3d::Identity();
    Eigen::MatrixXd Th = invDiffKinematicControlSimCompleteQuaternion(M, Kp, Kq, T, 0.0, Tf, DeltaT, scaleFactor, Tf, xe0, xef, q0, qf);
    

    // Output dei risultati
    std::cout << "Vettore di vettori di posizioni di articolazioni nel tempo:\n" << Th << std::endl;
    std::cout << "col: " << Th.cols() << ", row: " << Th.rows() << std::endl;


    return 0;
}


