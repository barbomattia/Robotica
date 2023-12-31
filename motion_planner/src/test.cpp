#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include <functional>


struct TransformationMatrices {
    Eigen::Matrix4d T10, T21, T32, T43, T54, T65, T60;
};

//CONFIGURAZIONE END EFFECTOR -----------------------------------------------------------------------------------------------
struct CinDir{
    Eigen::Vector3d pe;     //POSIZIONE END EFFECTOR
    Eigen::Matrix3d Re;     //ROTAZIONE END EFFECTOR
};


/*CINEMATICA DIRETTA --------------------------------------------------------------------------------------------------------- 
    PARAMETRI:
    - Th: vettore dei parametri q dei joints
    - scaleFactor: variabile che consente di adattare le dimensioni del modello del robot UR5
*/
CinDir CinematicaDiretta(const Eigen::VectorXd& Th, double scaleFactor) {
    /* Inizializzazione dei vettori A e D e del vettore Alpha con i valori del braccio robotico:

    A: vettore di tutti i parametri a del sistema DH dei link
    D: vettore di tutti i parametri d del sistema DH dei link
    Alpha: vettore di tutti i parametri alpha del sistema DH dei link

    A ed Alpha sono parametri costatni per loro natura, metre D lo consideriamo costante perchè non abbiamo prismatic joint

    Con A, D noi forniamo delle proporzioni che devono essere rispettate, le dimensioni vere e proprie vengono definite in base a scaleFactor */

    Eigen::VectorXd A(6), D(6), Alpha(6);
    A << 0 * scaleFactor, -0.425 * scaleFactor, -0.3922 * scaleFactor, 0, 0, 0;
    D << 0.1625 * scaleFactor, 0, 0, 0.1333 * scaleFactor, 0.0997 * scaleFactor, 0.0996 * scaleFactor;
    Alpha << M_PI / 2, 0, 0, M_PI / 2, -M_PI / 2, 0;

    //FUNZIONE per definire LE MATRICI DI TRASFORMAZIONE di un link 
    auto Tij = [](double th, double alpha, double d, double a) -> Eigen::Matrix4d {
        Eigen::Matrix4d result;
        result << cos(th), -sin(th) * cos(alpha), sin(th) * sin(alpha), a * cos(th),
                  sin(th), cos(th) * cos(alpha), -cos(th) * sin(alpha), a * sin(th),
                  0, sin(alpha), cos(alpha), d,
                  0, 0, 0, 1;
        return result;
    };

    //CALCOLO delle MATRICI DI TRASPORTO usando la funzione Tij appena definita
    TransformationMatrices Tm;   
    Tm.T10 = Tij(Th(0), Alpha(0), D(0), A(0));
    Tm.T21 = Tij(Th(1), Alpha(1), D(1), A(1));
    Tm.T32 = Tij(Th(2), Alpha(2), D(2), A(2));
    Tm.T43 = Tij(Th(3), Alpha(3), D(3), A(3));
    Tm.T54 = Tij(Th(4), Alpha(4), D(4), A(4));
    Tm.T65 = Tij(Th(5), Alpha(5), D(5), A(5));

    //CALCOLO LA MATRICE DI TRASPORTO FINALE 
    Tm.T60 = Tm.T10 * Tm.T21 * Tm.T32 * Tm.T43 * Tm.T54 * Tm.T65;

    //ESTRAGGO DALLA MATRICE T60 la POSIZIONE pe, e la ROTAZIONE Re dell'END EFFECTOR 
    Eigen::Vector3d pe = Tm.T60.block<3, 1>(0, 3);
    Eigen::Matrix3d Re = Tm.T60.block<3, 3>(0, 0);

    CinDir result;
    result.pe = pe;
    result.Re = Re;
    return result;
}


/*CINEMATICA INVERSA --------------------------------------------------------------------------------------------------------- 
    PARAMETRI:
    - p60: posizione end-effector
    - R60: rotazione end-effector
    - scaleFactor: variabile che consente di adattare le dimensioni del modello del robot UR5
*/
Eigen::MatrixXd cinematicaInversa(Eigen::Vector3d p60, Eigen::Matrix3d R60, double scaleFactor) {

    // Inizializzazione dei vettori A e D e del vettore Alpha con i valori del braccio robotico:
    Eigen::VectorXd A(6);
    A << 0, -0.425, -0.3922, 0, 0, 0;
    A *= scaleFactor;

    Eigen::VectorXd D(6);
    D << 0.1625, 0, 0, 0.1333, 0.0997, 0.0996;
    D *= scaleFactor;

    Eigen::VectorXd Alpha(6);
    Alpha << M_PI / 2, 0, 0, M_PI / 2, -M_PI / 2, 0;

    //FUNZIONE per definire LE MATRICI DI TRASFORMAZIONE di un link
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

/*FUNZIONE CALCOLO JACOBIANA ---------------------------------------------------------------------------------------------------
    PARAMETRI:
    - Th: vettore dei parametri q dei joints
    - scaleFactor: variabile che consente di adattare le dimensioni del modello del robot UR5
*/
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

//FUNZIONE per CALCOLO CONIUGATA di un QUATERNIONE -------------------------------------------------------------------------------------
Eigen::Quaterniond quatconj(const Eigen::Quaterniond& q) {
    return Eigen::Quaterniond(q.w(), -q.x(), -q.y(), -q.z());
}

//FUNZIONE per CALCOLO PRODOTTO tra QUATERNIONI --------------------------------------------------------------------------------------
Eigen::Quaterniond quatmultiply(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2) {
    return q1 * q2;
}

//FUNZIONE che RITORNA la PARTE VETTORIALE di un QUATERNION --------------------------------------------------------------------------
Eigen::Vector3d parts(const Eigen::Quaterniond& q) {
    return q.vec();
}


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
    double scaleFactor
) {
    // calcolo la Jacobiana per la configurazione attuale del braccio
    Eigen::MatrixXd J = ur5Jac(q, scaleFactor);

    // controllo che il determinate della jacobiano sia uguale a zero, in questo caso abbiamo a che fare con singolarità
    if (std::abs(J.determinant()) < 1e-3) {
        std::cerr << "vicino ad una singolarità" << std::endl;
        //implementazione
    }
    

    //Calcolo l'errore di orientazione: Δq=qd * qe.coniugato 
    Eigen::Quaterniond qp = qd * qe.conjugate();

    //Prendo la parte vettoriale del quaternioe errore 
    Eigen::Vector3d eo = qp.vec();

    //Calcolo la derivata di q: dotQ usando la formula
    Eigen::VectorXd dotQ = J.inverse() * (Eigen::VectorXd(6) << (vd + Kp * (xd - xe)), (omegad + Kq * eo)).finished();

    return dotQ;
}


/*INTERPOLAZIONE LINEARE tra due POSIZIONI dell'end effector su un tempo NORMALIZZATO ----------------------------------------
    PARAMETRI:
    - xe0: posizione iniziale end-effector
    - xef: posizione finale end-effector
    - tb: tempo attuale 
    - Tf: tempo totale a disposizione per il moto
*/
Eigen::MatrixXd pd(double tb, double Tf, Eigen::MatrixXd xe0, Eigen::MatrixXd xef) {

    double t = tb / Tf;             // calcolo t: valore del tempo normalizzato tra 0 (inizio movimento) ed 1(fine movimento)

    Eigen::MatrixXd result(3, 1);   // defnisco la matrice result

    if (t > 1) {                    // se t > 1 vuol dire che il tempo attuale a superato il tempo a disposizione
        result = xef;               // quindi ritono la posizione finale
    } else {
        result = t * xef + (1 - t) * xe0;   // altrimenti ritorno la nuova posizione interpolata tra xe0 e xef utilizzando l'interpolazione lineare
    }
    return result;
}


/*INTERPOLAZIONE LINEARE tra due ORIENTAZIONI dell'end effector su un tempo NORMALIZZATO ----------------------------------------
    PARAMETRI:
    - phie0: orientazione (angoli Eulero) iniziali end-effector
    - phief: orientazione (angoli Eulero) finali end-effector
    - tb: tempo attuale 
    - Tf: tempo totale a disposizione per il moto
*/
Eigen::MatrixXd phid(double tb, double Tf, Eigen::MatrixXd phief, Eigen::MatrixXd phie0) {
    
    double t = tb / Tf;             // calcolo t: valore del tempo normalizzato tra 0 (inizio movimento) ed 1(fine movimento)
    
    Eigen::MatrixXd result(3, 1);   // defnisco la matrice result
    
    if (t > 1) {                    // se t > 1 vuol dire che il tempo attuale a superato il tempo a disposizione
        result = phief;             // quindi ritono l'orientazione finale
    } else {
        result = t * phief + (1 - t) * phie0;       // altrimenti ritorno la nuova orientazione interpolata tra xe0 e xef utilizzando l'interpolazione lineare
    }
    return result;
}


/*INTERPOLAZIONE SFERICA tra due CONFIGURAZIONE dell'end effector su un tempo NORMALIZZATO ----------------------------------------
    PARAMETRI:
    - q0: quaternione della configurazione iniziali end-effector
    - phief: quaternione della configurazione finali end-effector
    - tb: tempo attuale 
    - Tf: tempo totale a disposizione per il moto
*/
Eigen::Quaterniond qd(double tb, double Tf, Eigen::Quaterniond q0, Eigen::Quaterniond qf) {

    double t = tb / Tf;             // calcolo t: valore del tempo normalizzato tra 0 (inizio movimento) ed 1(fine movimento)

    Eigen::Quaterniond result;      // defnisco la matrice result

    if (t > 1) {                    // se t > 1 vuol dire che il tempo attuale a superato il tempo a disposizione
        result = qf;                // quindi ritono il quaternione finale
    } else {
        result = q0.slerp(t, qf);   // altrimenti ritorno un nuovo quaternione derivato dall'interpolata sferica tra q0 e qf 
    }
    return result;
}


//FUNZIONE per CONVERTIRE una ORIENTAZIONE (angoli EULERO) in una MATRICE di ROTAZIONE ----------------------------------------------s
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


// FUNZIONE per CALCOLO CONFIGURAZIONI JOINT usando un Control basato sui QUATERNIONI e su INTERPOLAZIONE SFERICA --------------------
Eigen::MatrixXd invDiffKinematicControlSimCompleteQuaternion(
    const Eigen::VectorXd& TH0,     // vettore q che contiene le configurazioni iniziali di joint
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
    Eigen::Quaterniond qf           // quaternione finale end-effector
) {
    // Dichiaro la matrice q dove ogni riga corrisponde ad uno step temporale ed contiene le configurazioni dei joint in quel preciso step temporale
    std::vector<Eigen::VectorXd> q;     // q è un vettore di vettori, cioè una matrice

    int L = T.size();                   // L numero di step temporali
    Eigen::VectorXd qk = TH0;           // qk copia della configurazione iniziale dei joint
    q.push_back(qk);                    // qk pushata come prima riga in q

    // Per ogni step calcolo la configurazione dei joint 
    for (int i = 1; i < L - 1; ++i) {   

        // tramite la cinematica diretta trovo la configurazione dell'end effector all'inizio dello step
        auto result = CinematicaDiretta(qk, scaleFactor);   
        Eigen::VectorXd xe = result.pe;     // posizione end effector 
        Eigen::Matrix3d Re = result.Re;     // rotazione end effector 
        Eigen::Quaterniond qe(Re);          // calcolo il quaternione relativo alla rotazione end effector 

        // tramite la funzione pd calcola la posizione nel lasso temporale corrente T[i] e quello iniziale 0
        Eigen::MatrixXd vd1 = (pd(T[i], Tf, xe0, xef) - pd(0, Tf, xe0, xef));   // trova il delta posizione
        // calcola la velocità desiderata dello step basandosi sul delta spostamento ed il tempo a disposizione per lo step 
        Eigen::MatrixXd vd = vd1 / Dt;            

        Eigen::Quaterniond qd_t = qd(T[i], Tf, q0, qf);                 // quaternione ad inizio step
        Eigen::Quaterniond qd_t_plus_Dt = qd(T[i] + Dt, Tf, q0, qf);    // quaternione a fine step 

        Eigen::Quaterniond work = quatmultiply(qd_t_plus_Dt, qd_t.conjugate());    // moltiplico i due quaternioni per trovare il delta
        /* la riga sucessiva adattare la variazione angolare: work in modo proporzionale al passo temporale Dt. 
        Questo è necessario per garantire che la variazione angolare sia coerente con il passo temporale.*/
        work.coeffs() *= 2.0 / Dt;          

        /* un quaternione è visto come una combinazione di un vettore di rotazione e un termine scalare:
            - Il vettore di rotazione pè la parte immaginaria del quaternione e rappresenta l'asse di rotazione
            - Il termine scalare è l'angolo di rotazione attorno a quell'asse.

        Nel caso di work esso è un quaternione che rappresenta una variazioni tra quaternioni.
        In quaeto caso la sua parte vettoriale può essere interpretata come misura della variazione dell'asse di rotazione
        e quindi della velocità angolare attorno a quell'asse.
        omegad è la parte vettoriale del quaternione variazione cioè la velocita angolare desiderata */
        Eigen::Vector3d omegad = parts(work);   

        // calcolo la derivata del quaternione 
        Eigen::VectorXd dotqk = invDiffKinematiControlCompleteQuaternion(
            qk, xe, pd(T[i],Tf, xe0, xef), vd, omegad, qe, qd(T[i],Tf, q0, qf), Kp, Kq, scaleFactor
        );

        // calcolo con una funzione lineare le configurazioni dei joint a fine step e li inserisco nella matrice configurazioni q
        qk = qk + dotqk * Dt;
        q.push_back(qk);
    }

    
    // Convert the vector of vectors q to a matrix
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


