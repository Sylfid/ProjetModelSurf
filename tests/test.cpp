#include "../src/load_off_file.h"
#include "../src/vector3d.h"
#include <assert.h>
#include <sstream>


/**
  * \test
  * \brief on teste plein de choses : l'utilisation std::vector et l'affichage
            de la precision d'un double sur un flux (pour notre apprentissage).
*/
int main () {

    std::vector<Vector3d> cloud;
    cloud.resize(3);

    Vector3d v1 = Vector3d(0, 1, 2);
    Vector3d v2 = Vector3d(3, 4, 5);
    Vector3d v3 = Vector3d(6, 7, 8);

    double v1x = v1.getX();

    cloud[0] = v1;
    cloud[1] = v2;
    cloud[2] = v3;
    std::cout << "mon nuage de point : " << std::endl;
    for (std::size_t i=0 ; i<cloud.size() ; i++) {
        cloud[i].display(std::cout);
    }
    std::cout << "un point v1 de mon nuage :" << std::endl;
    v1.display(std::cout);

    assert(cloud[0].getX() == v1.getX());
    assert(cloud[0].getY() == v1.getY());
    assert(cloud[0].getZ() == v1.getZ());

    assert(cloud[1].getX() == v2.getX());
    assert(cloud[1].getY() == v2.getY());
    assert(cloud[1].getZ() == v2.getZ());

    assert(cloud[2].getX() == v3.getX());
    assert(cloud[2].getY() == v3.getY());
    assert(cloud[2].getZ() == v3.getZ());

    printf("taille de lechantillon : %ld\n", cloud.size());
    assert(cloud.size() == 3);

    v1.setX(-6.2);
    printf("changement de valeur v1 : \n");
    v1.display(std::cout);

    printf("j'espère que ca n'affecte pas mon cloud...\n");

    for (std::size_t i=0 ; i<cloud.size() ; i++) {
        cloud[i].display(std::cout);
    }

    assert(cloud[0].getX() != v1.getX());
    assert(cloud[0].getX() == v1x);

    printf("\ntest du resize (5) : \n");
    cloud.resize(5);
    for (std::vector<Vector3d>::iterator it = cloud.begin() ; it != cloud.end() ; ++it) {
        (*it).display(std::cout);
    }

    assert(cloud[0].getX() == v1x);
    assert(cloud[0].getY() == v1.getY());
    assert(cloud[0].getZ() == v1.getZ());

    assert(cloud[1].getX() == v2.getX());
    assert(cloud[1].getY() == v2.getY());
    assert(cloud[1].getZ() == v2.getZ());

    assert(cloud[2].getX() == v3.getX());
    assert(cloud[2].getY() == v3.getY());
    assert(cloud[2].getZ() == v3.getZ());

    for (std::size_t k=3 ; k<cloud.size() ; k++) {
        for(int i=0 ; i<3 ; i++)
             assert(cloud[k][i] == 0);
    }

    double m=3.141592653589793238462;
    std::cout.precision(std::numeric_limits<double>::digits10 + 1);
    printf("\ntest affichage de la precision d'un double:\n");
    std::cout << m << std::endl;

    return 0;
}
