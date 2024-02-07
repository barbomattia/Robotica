#include "../include/Kinematic.h"

/*
int main(){

    Eigen::Quaterniond q1(1.0, 2.0, 3.0, 4.0);
    Eigen::Quaterniond q2(5.0, 6.0, 7.0, 8.0);

    Eigen::Quaterniond result1 = q1*q2;

    std::cout << std::endl << "q1: " << q1 << std::endl;
    std::cout << "q2: " << q2 << std::endl << std::endl;
    std::cout << "result: " << result1 << std::endl;

    Eigen::Quaterniond result2 = q1*q2.conjugate();
    std::cout << std::endl << "q1: " << q1 << std::endl;
    std::cout << "q2: " << q2.conjugate() << std::endl << std::endl;
    std::cout << "result: " << result2 << std::endl;


    Eigen::Quaterniond qa = slerpFunction(q1, q2, 0.3);
    Eigen::Quaterniond qb = q1.slerp(0.3, q2);

    std::cout << std::endl << "slerp metodo: " << qb << std::endl;

    std::cout << std::endl << "slerp funzione: " << qa << std::endl;
    
    







    return 0;
}
*/

int main() {

    /*
   // Inizializzazione delle variabili globali
    Eigen::MatrixXd xe0(3, 1);
    Eigen::MatrixXd xef(3, 1);
    Eigen::MatrixXd phie0(3, 1);
    Eigen::MatrixXd phief(3, 1);
    double scaleFactor = 1.0;
    double Tf = 10.0; 
    double DeltaT = 2;
    Eigen::VectorXd T;
    T = Eigen::VectorXd::LinSpaced(static_cast<int>((Tf / DeltaT) + 1), 0, Tf);
     
    //inizializzazione paramateri     
    xe0 << 0.3, 0.3, 1;                       //posizione attuale end-effector
    xe0 *= scaleFactor;
    phie0 << 0, 0, 0;
    
    xef << 0.5, 0.4, 1.2;                   //posizione finale end-effector
    xef *= scaleFactor;
    phief << M_PI / 2, M_PI / 2, M_PI / 2;

    //stampe punti
    //std::cout << "punto iniziale: " << xe0.transpose() << std::endl << std::endl;
    //std::cout << "punto finale: " << xef.transpose() << std::endl << std::endl;

    Eigen::Matrix3d workM;
    workM = euler2RotationMatrix(phie0, "XYZ");
    Eigen::Quaterniond q0(workM);

    Eigen::Matrix4d Tt0 = Eigen::Matrix4d::Identity();
    Tt0.block<3, 3>(0, 0) = euler2RotationMatrix(phief, "XYZ");
    Tt0.block<3, 1>(0, 3) = xef;

    Eigen::Quaterniond qf(Tt0.block<3, 3>(0, 0)); 

    //std::cout  << "Quaternione Iniziale: " << q0 << std::endl;
    //std::cout  << "Quaternione Finale: " << qf << std::endl;

    // Chiamata alle funzioni
    Eigen::MatrixXd TH0 = cinematicaInversa(pd(0, Tf, xe0, xef), euler2RotationMatrix(phid(0, Tf, phief, phie0), "XYZ"), scaleFactor);
    int count = 0;
 
    std::cout << std::endl << "CINEMATICA INVERSA: " << std::endl << TH0 << std::endl << std::endl;  

    Eigen::Matrix3d Kp = 3 * Eigen::Matrix3d::Identity();
    Eigen::Matrix3d Kq = -2.65 * Eigen::Matrix3d::Identity();
    
    for(int i=0; i<8; i++){
        
        NaNColumn M = getFirstColumnWithoutNaN(TH0);
        bool check = M.isNaN;
        std::cout << std::endl << "CONF: " << i+1 << std::endl << TH0 << std::endl; 
        
        if(check){
            Eigen::MatrixXd Th = invDiffKinematicControlSimCompleteQuaternion(M.configurazione, Kp, Kq, T, 0.0, Tf, DeltaT, scaleFactor, Tf, xe0, xef, q0, qf);
        
            for(int j = 0; j<Th.rows(); j++){
                
                Eigen::MatrixXd giunti = posizioneGiunti(Th.row(j), scaleFactor);

                std::cout << std::endl << "CONF: " << i+1 << ", STEP: " << j+1 << std::endl;
                std::cout << "configurazione: " << Th.row(j) << std::endl << std::endl;
                std::cout << "posizione giunti funzione: " << std::endl << giunti << std::endl << std::endl;          
                
                if(checkCollisioni(giunti, 0.025, 0.05, scaleFactor)){
                    std::cout << "congifurazione con collisione" << std::endl;
                    check = false;
                    break;
                }
            }

            if(check){
                CinDir EndEffectorFinal = CinematicaDiretta(Th.row(Th.rows()-1), scaleFactor);
                Eigen::Quaterniond qef(EndEffectorFinal.Re);

                std::cout << "MATRICE TH" << std::endl << Th << std::endl;
                std::cout << std::endl << "Posizione end effector finale: " << EndEffectorFinal.pe.transpose() << std::endl;
                std::cout << std::endl << "Orientamento end effector finale: " << std::endl << EndEffectorFinal.Re << std::endl;
                break;
            } 
        }    
    }
    
    */

   Eigen::MatrixXd mat1(3, 2);
    mat1 << 1, 2,
            3, 4,
            5, 6;

    Eigen::MatrixXd mat2(3, 3);
    mat2 << 7, 8, 9,
            10, 11, 12,
            13, 14, 15;

    // Creazione di una nuova matrice grande abbastanza da contenere entrambe le matrici
    Eigen::MatrixXd mergedMat(mat1.rows(), mat1.cols() + mat2.cols());

    // Copia mat1 nella parte sinistra di mergedMat
    mergedMat.block(0, 0, mat1.rows(), mat1.cols()) = mat1;

    // Copia mat2 nella parte destra di mergedMat
    mergedMat.block(0, mat1.cols(), mat2.rows(), mat2.cols()) = mat2;

    std::cout << "Matrice unita:\n" << mergedMat << std::endl;

    return 0;
    
    


    return 0;
}









