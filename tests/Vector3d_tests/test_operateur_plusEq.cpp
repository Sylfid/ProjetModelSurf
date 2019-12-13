#include "../../src/vector3d.h"
#include <stdio.h>
#include <assert.h>

/**
  * \file test_operateur_plusEq.cpp
  * \brief Programme de test de l'opérateur +=
*/
int main() {

    Vector3d v = Vector3d(5,1,2);
    v += 2.7;
    assert(v.getX() == 7.7);
    assert(v.getY() == 3.7);
    assert(v.getZ() == 4.7);

    Vector3d v1 = Vector3d(1.3,3.2,0);
    v += v1;
    assert(v.getX() == 9.0);
    assert(v.getY() == 6.9);
    assert(v.getZ() == 4.7);

    return 0;
}
