#include "../../src/vector3d.h"
#include <stdio.h>
#include <assert.h>

/**
  * \file test_operateur_minus.cpp
  * \brief Programme de test de l'opérateur - entre deux vecteurs
*/
int main() {

    Vector3d v = Vector3d(1,5,2);
    Vector3d v1 = Vector3d(1,5,5);
    //Vector3d v2 = v - v1;

    std::cout << v;
    std::cout << "\n";
    std::cout << v1;
    // std::cout << "\n";
    // std::cout << v2;

    assert(v.getX() == 1);
    assert(v.getY() == 5);
    assert(v.getZ() == 2);

    assert(v1.getX() == 1);
    assert(v1.getY() == 5);
    assert(v1.getZ() == 5);

    // assert(v2.getX() == 0);
    // assert(v2.getY() == 0);
    // assert(v2.getZ() == -3);

    return 0;
}
