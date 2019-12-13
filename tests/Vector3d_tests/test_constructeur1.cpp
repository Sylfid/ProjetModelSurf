#include "../../src/vector3d.h"
#include <stdio.h>
#include <assert.h>
#include <sstream>

/**
  * \test
  * \brief Programme de test de classe Vector3d
*/
int main() {

    std::cout << "Initialisation du premier vecteur :\n";
    Vector3d v(1.,3.,1.);
    std::stringstream str;
    assert(v.getX() == 1.);
    assert(v.getY() == 3.);
    assert(v.getZ() == 1.);
    std::cout << "OK\n";

    std::cout << "Initialisation du deuxieme vecteur :\n";
    Vector3d x(1,5,9);
    std::stringstream str1;
    x.display(str1);
    assert(str1.str() == "1\t5\t9\n");
    std::cout << "OK\n";

    return 0;
}
