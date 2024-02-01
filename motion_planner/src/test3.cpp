#include "../include/Kinematic.h"

int main() {

    double scaleFactor = 10.0;
    Eigen::VectorXd th;
    th.resize(6);
    th << 3.607390, -0.275293, 2.119540, 2.868140, 1.570800, -2.036590;


    CinDir result = CinematicaDiretta(th, scaleFactor);

    std::cout << "result Direct Cinematic of th: " << th.transpose() << std::endl;
    std::cout << "pe: " << result.pe.transpose()<< std::endl;
    std::cout << "Re: " << std::endl << result.Re << std::endl;

}