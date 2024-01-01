#include "../include/Kinematic.h"

int main() {

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
