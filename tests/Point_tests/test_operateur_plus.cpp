#include "../../src/point.h"
#include <stdio.h>
#include <assert.h>

/**
  * \file test_operateur_plus.cpp
  * \brief Programme de test de l'opérateur + entre deux vecteurs
*/
int main() {

    Point v = Point(2.,2.,2.);
    Point v1 = Point(5.,5.,5.);
    Point v2 = v + v1;
    assert(v2.getX() == 7.);
    assert(v2.getY() == 7.);
    assert(v2.getZ() == 7.);

    return 0;
}
