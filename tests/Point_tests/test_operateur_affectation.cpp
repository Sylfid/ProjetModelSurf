#include "../../src/point.h"
#include <stdio.h>
#include <assert.h>
#include <sstream>

/**
  * \file test_operateur_affectation.cpp
  * \brief Programme de test de l'opérateur d'affectation
  *
  * \details Programme de test pour l'opérateur d'affectation =
*/
int main() {

    Point v;
    v = Point(3,1,1);

    assert(v.getX() == 3);
    assert(v.getY() == 1);
    assert(v.getZ() == 1);

    return 0;
}
