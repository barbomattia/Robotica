#include "../include/Kinematic.h"

Eigen::Quaterniond slerpFunction(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2, double t) {
    // Normalizzazione dei quaternioni
    Eigen::Quaterniond quat1 = q1.normalized();
    Eigen::Quaterniond quat2 = q2.normalized();

    // Calcolo del prodotto scalare tra i quaternioni
    double dotProduct = quat1.coeffs().dot(quat2.coeffs());

    // Se il prodotto scalare è negativo, inverti uno dei quaternioni
    if (dotProduct < 0) {
        quat1.coeffs() = -quat1.coeffs();
        dotProduct = -dotProduct;
    }

    // Calcolo dell'angolo tra i quaternioni
    double theta = acos(dotProduct);
    double sinTheta = sin(theta);

    // Calcolo dell'interpolazione sferica
    Eigen::Quaterniond result = Eigen::Quaterniond(
        (sin((1 - t) * theta) / sinTheta) * quat1.coeffs() + (sin(t * theta) / sinTheta) * quat2.coeffs()
    );

    // Normalizzazione del risultato
    return result.normalized();
}

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




