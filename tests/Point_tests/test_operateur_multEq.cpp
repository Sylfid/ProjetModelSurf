#include "../../src/point.h"
#include <stdio.h>
#include <assert.h>

/**
  * \file test_operateur_multEq.cpp
  * \brief Programme de test de l'opérateur *=
*/
int main() {

    /* --------------------- double ----------------------- */
    Point v = Point(5.,1.,2.);
    v *= 10.;
    assert(v.getX() == 50.);
    assert(v.getY() == 10.);
    assert(v.getZ() == 20.);
    return 0;
}
