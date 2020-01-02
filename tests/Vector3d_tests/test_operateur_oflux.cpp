#include "../../src/vector3d.h"
#include <stdio.h>
#include <sstream>
#include <assert.h>

/**
  * \file test_operateur_oflux.cpp
  * \brief Programme de test de l'opérateur de flux
*/
int main() {

    Vector3d v = Vector3d(3.5,0,2.5);
    std::stringstream str;
    v.display(str);
    assert(str.str() == "3.5\t0\t2.5\n");

    return 0;
}
