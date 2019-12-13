#include "../../src/vector3d.h"
#include <stdio.h>
#include <assert.h>
#include <sstream>

/**
  * \file test_operateur_divEq.cpp
  * \brief Programme de test de l'opérateur /=
*/
int main() {

    Vector3d v = Vector3d(6,2,10);
    v /= 2;
    std::cout << v;
    assert(v.getX() == 3);
    assert(v.getY() == 1);
    assert(v.getZ() == 5);

    return 0;
}
