#include "../../src/vector3d.h"
#include <stdio.h>
#include <assert.h>
#include <sstream>

/**
  * \test
  * \brief Programme de test de classe point
  *
  * \details Programme de test du constructeur par recopie
*/
int main() {

    Vector3d v(1.8,1.8,1.8);
    std::stringstream str;
    v.display(str);
    assert(str.str() == "1.8\t1.8\t1.8\n");
    std::cout << "initialisation OK\n\n";

    std::cout << "Initialisation d'un vecteur par recopie :\n";
    Vector3d copie(v);
    std::stringstream str1;
    copie.display(str1);
    assert(v.getX() == 1.8);
    assert(v.getY() == 1.8);
    assert(v.getZ() == 1.8);
    std::cout << "recopie OK\n";

    return 0;
}
