#include "../../src/vector3d.h"
#include <stdio.h>
#include <sstream>
#include <assert.h>
#include <math.h>

/**
  * \file test_operateur_minusEq.cpp
  * \brief Programme de test de l'opérateur -=
*/
int main() {

    Vector3d v = Vector3d(2.3,2.3,2.3);
    v -= 2;
    double eps = 0.0000000001;
    assert(fabs(v.getX()-0.3) < eps);
    assert(fabs(v.getY()-0.3) < eps);
    assert(fabs(v.getZ()-0.3) < eps);

    Vector3d v1 = Vector3d(0.,0.,0.);
    Vector3d v2 = Vector3d(1.,1.,1.);
    v1 -= v2;
    assert(fabs(v1.getX()+1) < eps);
    assert(fabs(v1.getY()+1) < eps);
    assert(fabs(v1.getZ()+1) < eps);


    return 0;
}
