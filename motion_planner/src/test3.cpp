#include "../include/Kinematic.h"

int main() {

    double scaleFactor = 10.0;
    Eigen::VectorXd th;
    th.resize(6);
    th << 3.607390, -0.275293, 2.119540, 2.868140, 1.570800, -2.036590;


    Eigen::VectorXd wD = wDerived(th, scaleFactor);
    std::cout << "PROVA " << wD.transpose() << std::endl << std::endl;

    return 0;

}