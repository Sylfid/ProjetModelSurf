#include "../../src/vector3d.h"
#include <stdio.h>
#include <assert.h>

/**
  * \file test_operateur_plus.cpp
  * \brief Programme de test de l'opérateur + entre deux vecteurs
*/
int main() {

    Vector3d v = Vector3d(2.,2.,2.);
    Vector3d v1 = Vector3d(5.,5.,5.);
    Vector3d v2 = v + v1;
    assert(v2.getX() == 7.);
    assert(v2.getY() == 7.);
    assert(v2.getZ() == 7.);

    return 0;
}
