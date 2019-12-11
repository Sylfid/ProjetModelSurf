#include "../../src/point.h"
#include <stdio.h>
#include <assert.h>
#include <sstream>

/**
  * \file test_affichage.cpp
  * \brief Programme de test de classe Point
  *
  * \details Programme de test pour la méthode d'affichage
                du contenu d'un vecteur : display
*/
int main() {

    Point v(3,1,2.5);
    std::stringstream str;
    v.display(str);
    v.display(std::cout);
    std::cout << "\n";
    assert(str.str() == "3\t1\t2.5\n");

    return 0;
}
