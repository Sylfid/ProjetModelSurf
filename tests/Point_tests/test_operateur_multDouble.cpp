#include "../../src/point.h"
#include <stdio.h>
#include <assert.h>

/**
  * \file test_operateur_multDouble.cpp
  * \brief Programme de test de l'opérateur + entre un vecteur et un double
*/
int main() {

    /* --------------------- double ----------------------- */

    Point v = Point(5,1,5);
    Point v1 = 2.*v;
    Point v2 = 3.0*v;

    assert(v1.getX() == 10.);
    assert(v1.getY() == 2.);
    assert(v1.getZ() == 10.);

    assert(v2.getX() == 15.);
    assert(v2.getY() == 3.);
    assert(v2.getZ() == 15.);

    return 0;
}
