#include "../../src/vector3d.h"
#include <stdio.h>
#include <assert.h>
#include <sstream>
#include <math.h>

/**
  * \file test_accesseur.cpp
  * \brief Programme de test de classe Vector3d
  *
  * \details Programme de test pour la méthode acccesseur (int i)
*/
int main() {

    Vector3d v(3,1,2.5);
    assert(v[0] == 3);
    assert(v[1] == 1);
    assert(v[2] == 2.5);

    double d = v[2];
    d = 10;
    assert(d == 10);
    assert(v[2] == 2.5);

    //
    Vector3d v1(10,-9,2);
    v1 -= v;
    double eps = 0.00000000000001;
    assert(fabs(v1[0]-7) < eps);
    assert(fabs(v1[1]+10) < eps);
    assert(fabs(v1[2]+0.5) < eps);

    return 0;
}
