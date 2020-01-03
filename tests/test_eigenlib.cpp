#include "../src/plane.h"
#include "../src/vector3d.h"

#include <vector>
#include <Eigen/Eigen>
#include <Eigen/Dense>

/**
  * \test
  * \brief On teste l'utilisation de la bibliothèque Eigen (pour notre
            apprentissage) : valeurs propres et vecteurs propres d'une matrice
            symétrique réelle (tirée de la doc de Eigen).
            Ce programme teste une partie de la méthode Cloud::compute_normal.
*/
int main () {
    Eigen::MatrixXd AA(3,3);
    AA <<   1, 2, 3,
            4, 5, 6,
            7, 8, 9;

    std::cout << "Here is a random 3x3 matrix, AA:" << std::endl << AA << std::endl << std::endl;
    // A SYMETRIQUE REELLE !!
    Eigen::MatrixXd A = Eigen::MatrixXd::Random(3,3);
    A(1,1) = 0.2620;
    A(0,1) = A(1,0);
    A(0,2) = A(2,0);
    A(2,1) = A(1,2);

    std::cout << "Here is a random 3x3 matrix, A:" << std::endl << A << std::endl << std::endl;
    Eigen::EigenSolver<Eigen::MatrixXd> es(A);

    // third eigen vector = the min
    int pos_min = 0;
    int pos_max = 0;
    double min = es.eigenvalues()[0].real();
    double max = es.eigenvalues()[0].real();
    for (int i = 1; i < 3; i++) {
        double eigenvalue = es.eigenvalues()[i].real();
        if (eigenvalue < min) {
            min = eigenvalue;
            pos_min = i;
        }
        if (eigenvalue >= max) {
            max = eigenvalue;
            pos_max = i;
        }
    }
    int pos = 3 - (pos_min + pos_max);

    std::cout << "pos min : " << pos_min << std::endl;
    std::cout << "pos max : " << pos_max << std::endl;
    std::cout << "autre : " << pos << std::endl << std::endl;

    std::cout << "Decomposition : \n" << std::endl;

    std::cout << "The eigenvalues : " << std::endl << es.eigenvalues() << std::endl;

    Eigen::VectorXcd v1 = es.eigenvectors().col(pos_max);
	Eigen::VectorXcd v2 = es.eigenvectors().col(pos);
    Eigen::VectorXcd v3 = es.eigenvectors().col(pos_min);
    std::cout << "The first eigenvector of the 3x3 matrix of A is:"
     << std::endl << v1 << std::endl << std::endl;
    std::cout << "The second eigenvector of the 3x3 matrix of A is:"
      << std::endl << v2 << std::endl;
    std::cout << "The third eigenvector of the 3x3 matrix of A is:"
       << std::endl << v3 << std::endl;

    std::cout << std::endl << "Pseudo-decomposition : " << std::endl;
    Eigen::MatrixXd D = es.pseudoEigenvalueMatrix();
    Eigen::MatrixXd V = es.pseudoEigenvectors();

    Eigen::MatrixXd A_dec = V * D * V.inverse();

    std::cout << "The pseudo-eigenvalue matrix D is:" << std::endl << D << std::endl;
    std::cout << "The pseudo-eigenvector matrix V is:" << std::endl << V << std::endl;
    std::cout << "Finally, V * D * V^(-1) = " << std::endl << V * D * V.inverse() << std::endl;

    // normal vector
    Vector3d normal(v3(0).real(),v3(1).real(),v3(2).real());
    Vector3d u(v1(0).real(),v1(1).real(),v1(2).real());
    Vector3d v(v2(0).real(),v2(1).real(),v2(2).real());

    Vector3d c(0,0,0);
    // Plane P(c, normal, u ,v);
    Plane P;
    P.setCenter(c);
    P.setNormal(normal);
    P.setU(u);
    P.setV(v);

    std::cout << "\nAffichage de mon plan tangent test : " << std::endl;
    P.display(std::cout);
}
