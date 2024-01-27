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
    
    
    //for(int i=0; i<8; i++){
        Eigen::VectorXd M = getFirstColumnWithoutNaN(TH0);
        Eigen::MatrixXd posGiunti = posizioneGiunti(M, scaleFactor); 
        Eigen::Matrix3d Kp = 10.0 * Eigen::Matrix3d::Identity();
        Eigen::Matrix3d Kq = -10.0 * Eigen::Matrix3d::Identity();
        Eigen::MatrixXd Th = invDiffKinematicControlSimCompleteQuaternion(M, Kp, Kq, T, 0.0, Tf, DeltaT, scaleFactor, Tf, xe0, xef, q0, qf);
        int count = 0;

        for(int j=0; j<Th.rows(); j++){
            Eigen::VectorXd configurazione = Th.row(j);
            Eigen::MatrixXd temp = posizioneGiunti(configurazione, scaleFactor);
            count++;
            //if(checkCollisioni(temp, vertici tavolo)){ 
                //break;
            //} else {
                //ritorna matrice Th con le posizioni nel tempo da passare al robot
            //}   
        } 
    //} 

    std::cout << "Vettore di vettori di posizioni di articolazioni nel tempo:\n" << Th << std::endl;
    std::cout << "col: " << Th.cols() << ", row: " << Th.rows() << std::endl;
    std::cout << "count: " <<  count << std::endl;
   

   /*
   Eigen::VectorXd Th(6);
    Th << 0.1,0.4,0.3,0.4,0.5,0.6;
    double scaleFactor = 10.0;


    TransformationMatrices Tm;
    std::cout << "configurazione iniziale: " << Th.transpose() << std::endl << std::endl;

    std::cout << "CINEMATICA DIRETTA" << std::endl;
    CinDir result = CinematicaDiretta(Th, scaleFactor);

    std::cout << "vettore (pe): " << result.pe.transpose() << std::endl;
    std::cout << "matrice orientamento (Re): " << std::endl << result.Re << std::endl;

    std::cout << std::endl;
    
    //CINEMATICA INVERSA
    std::cout << "CINEMATICA INVERSA" << std::endl;
    //Vector3d p60(3.0, 3.0, 3.0);  
    //Matrix3d R60 = Eigen::Matrix3d::Identity();   
    Eigen::MatrixXd M = cinInversa(result.pe, result.Re, scaleFactor);

    Eigen::MatrixXd giunti = posizioneGiunti(M.col(0), scaleFactor);
    
    std::cout << "Matrice delle variabili congiunte M:\n" << M << std::endl << std::endl;

    std::cout << "Configurazione: " << 1 << std::endl;
    std::cout << giunti << std::endl << std::endl;
    */

    return 0;
}
