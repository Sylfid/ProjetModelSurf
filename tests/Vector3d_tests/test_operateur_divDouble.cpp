#include "../../src/vector3d.h"
#include <stdio.h>
#include <assert.h>

/**
  * \file test_operateur_divDouble.cpp
  * \brief Programme de test de l'opérateur * entre un vecteur et un double
*/
int main() {

    Vector3d v = Vector3d(5,5,30);
    Vector3d v1 = (0.5)*v;
    assert(v1.getX() == 2.5);
    assert(v1.getY() == 2.5);
    assert(v1.getZ() == 15.);

    return 0;
}
