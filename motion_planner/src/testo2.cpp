#include "../include/Kinematic.h"


int main() {

   // Inizializzazione delle variabili globali
    Eigen::MatrixXd xe0(3, 1);
    Eigen::MatrixXd xef(3, 1);
    Eigen::MatrixXd phie0(3, 1);
    Eigen::MatrixXd phief(3, 1);
    double scaleFactor = 10.0;
    double Tf = 10.0; 
    double DeltaT = 1;
    Eigen::VectorXd T;
    T = Eigen::VectorXd::LinSpaced(static_cast<int>((Tf / DeltaT) + 1), 0, Tf);
     
    //inizializzazione paramateri     
    xe0 << 0.3, 0.3, 0.0;                       //posizione attuale end-effector
    xe0 *= scaleFactor;
    phie0 << M_PI / 3, M_PI / 3, M_PI / 3;
    
    xef << 0.5, 0.4, 0.9;                   //posizione finale end-effector
    xef *= scaleFactor;
    phief << M_PI / 2, M_PI / 2, M_PI / 2;

    //stampe punti
    std::cout << "punto iniziale: " << std::endl << xe0.transpose() << std::endl << std::endl;
    std::cout << "punto finale: " << std::endl << xef.transpose() << std::endl << std::endl;
    
    Eigen::Matrix3d workM;
    workM = euler2RotationMatrix(phie0, "XYZ");
    Eigen::Quaterniond q0(workM);

    Eigen::Matrix4d Tt0 = Eigen::Matrix4d::Identity();
    Tt0.block<3, 3>(0, 0) = euler2RotationMatrix(phief, "XYZ");
    Tt0.block<3, 1>(0, 3) = xef;

    Eigen::Quaterniond qf(Tt0.block<3, 3>(0, 0)); 

    // Chiamata alle funzioni
    Eigen::MatrixXd TH0 = cinematicaInversa(pd(0, Tf, xe0, xef), euler2RotationMatrix(phid(0, Tf, phief, phie0), "XYZ"), scaleFactor);
    int count = 0;
 
    std::cout << "cinematica inversa: " << std::endl << TH0 << std::endl << std::endl;  
    
    Eigen::VectorXd M = getFirstColumnWithoutNaN(TH0);
    Eigen::Matrix3d Kp = 1.0 * Eigen::Matrix3d::Identity();
    Eigen::Matrix3d Kq = -1.0 * Eigen::Matrix3d::Identity();
    Eigen::MatrixXd Th = invDiffKinematicControlSimCompleteQuaternion(M, Kp, Kq, T, 0.0, Tf, DeltaT, scaleFactor, Tf, xe0, xef, q0, qf);

    std::cout << "Vettore di vettori di posizioni di articolazioni nel tempo:\n" << Th << std::endl;
     
    //stamoe configurazioni e coordinate giunti
    std::cout << std::endl;
    std::cout << "configurazione iniziale: " << std::endl << M.transpose() << std::endl << std::endl;  
    std::cout << "configurazione finale: " << std::endl << Th.row(9).transpose() << std::endl << std::endl;  
    
    Eigen::MatrixXd giuntiInizliale = posizioneGiunti(M, scaleFactor);
    Eigen::MatrixXd giuntiFinale = posizioneGiunti(Th.row(9), scaleFactor);
    std::cout << "posizione giunti iniziale: " << std::endl << giuntiInizliale << std::endl << std::endl;   
    std::cout << "posizione giunti finale: " << std::endl << giuntiFinale << std::endl << std::endl;  



    return 0;
}
