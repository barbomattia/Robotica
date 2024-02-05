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
     
    //inizializzazione paramateri     
    xe0 << 0.3, 0.3, 0.0;                       //posizione attuale end-effector
    xe0 *= scaleFactor;
    phie0 << 0, 0, 0;
    
    xef << 0.5, 0.4, 0.6;                   //posizione finale end-effector
    xef *= scaleFactor;
    phief << M_PI / 2, M_PI / 2, M_PI / 2;

    //stampe punti
    std::cout << "punto iniziale: " << xe0.transpose() << std::endl << std::endl;
    std::cout << "punto finale: " << xef.transpose() << std::endl << std::endl;

    Eigen::Matrix3d workM;
    workM = euler2RotationMatrix(phie0, "XYZ");
    Eigen::Quaterniond q0(workM);

    Eigen::Matrix4d Tt0 = Eigen::Matrix4d::Identity();
    Tt0.block<3, 3>(0, 0) = euler2RotationMatrix(phief, "XYZ");
    Tt0.block<3, 1>(0, 3) = xef;

    Eigen::Quaterniond qf(Tt0.block<3, 3>(0, 0)); 

    std::cout  << "Quaternione Iniziale: " << q0 << std::endl;
    std::cout  << "Quaternione Finale: " << qf << std::endl;

    // Chiamata alle funzioni
    Eigen::MatrixXd TH0 = cinematicaInversa(pd(0, Tf, xe0, xef), euler2RotationMatrix(phid(0, Tf, phief, phie0), "XYZ"), scaleFactor);
    int count = 0;
 
    // std::cout << "cinematica inversa: " << std::endl << TH0 << std::endl << std::endl;  

    Eigen::Matrix3d Kp = 3 * Eigen::Matrix3d::Identity();
    Eigen::Matrix3d Kq = -2.65 * Eigen::Matrix3d::Identity();
    
    for(int i=0; i<8; i++){
        NaNColumn M = getFirstColumnWithoutNaN(TH0);
        Eigen::MatrixXd Th = invDiffKinematicControlSimCompleteQuaternion(M.configurazione, Kp, Kq, T, 0.0, Tf, DeltaT, scaleFactor, Tf, xe0, xef, q0, qf);
        
        int count = 0;

        for(int j = 0; j<Th.rows(); j++){
            Eigen::MatrixXd giunti = posizioneGiunti(Th.row(j), scaleFactor);
            if(checkCollisioni(giunti, 0.02, scaleFactor)){
                std::cout << "congifurazione con collisione" << std::endl;
                count = 0;
                break;
            } else {
                count++;
            }
        }
        if(count == 600){
            std::cout << "count: " << count << std::endl;
            std::cout << "Vettore di vettori di posizioni di articolazioni nel tempo:\n" << Th << std::endl;
     
            //stamoe configurazioni e coordinate giunti
            std::cout << std::endl;
            // std::cout << "configurazione iniziale: " << std::endl << M.transpose() << std::endl << std::endl;  
            std::cout << "configurazione finale: " << std::endl << Th.row(99) << std::endl << std::endl;

            CinDir EndEffectorFinal = CinematicaDiretta(Th.row(99), scaleFactor);
            Eigen::Quaterniond qef(EndEffectorFinal.Re);
            std::cout << "Posizione end effector raggiunta: " << EndEffectorFinal.pe.transpose() << std::endl;
            std::cout << "Rotazione end effector raggiunta: \n" << EndEffectorFinal.Re << std::endl;  

            std::cout << "Quaternione end effector raggiunta: \n" << qef << std::endl;   
            
            Eigen::MatrixXd giuntiInizliale = posizioneGiunti(M.configurazione, scaleFactor);
            Eigen::MatrixXd giuntiFinale = posizioneGiunti(Th.row(9), scaleFactor);
            // std::cout << "posizione giunti iniziale: " << std::endl << giuntiInizliale << std::endl << std::endl;   
            // std::cout << "posizione giunti finale: " << std::endl << giuntiFinale.transpose() << std::endl << std::endl;  
        } else {
            std::cout << "nessuna configurazione valida" << std::endl;
        }
    }

    
    

    



    return 0;
}
