#include "../../src/vector3d.h"
#include <stdio.h>
#include <assert.h>
#include <sstream>

/**
  * \file test_affichage.cpp
  * \brief Programme de test de classe Vector3d
  *
  * \details Programme de test pour la méthode d'affichage
                du contenu d'un vecteur : display
*/
int main() {

    Vector3d v(3,1,2.5);
    std::stringstream str;
    v.display(str);
    v.display(std::cout);
    std::cout << "\n";
    assert(str.str() == "3\t1\t2.5\n");

    return 0;
}
