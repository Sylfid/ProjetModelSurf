#include "../../src/vector3d.h"
#include <stdio.h>
#include <sstream>
#include <assert.h>

/**
  * \file test_operateur_minusEq.cpp
  * \brief Programme de test de l'opérateur -=
*/
int main() {

    Vector3d v = Vector3d(2.3,2.3,2.3);
    v -= 2.;
    std::stringstream str;
    v.display(str);
    assert(str.str() == "0.3\t0.3\t0.3\n");

    Vector3d v1 = Vector3d(0.,0.,0.);
    Vector3d v2 = Vector3d(1.,1.,1.);
    v1 -= v2;
    std::stringstream str1;
    v1.display(str1);
    assert(v1.getX() == -1.);
    assert(v1.getX() == -1.);
    assert(v1.getX() == -1.);


    return 0;
}
