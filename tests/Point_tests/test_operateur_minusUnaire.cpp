#include "../../src/point.h"
#include <stdio.h>
#include <assert.h>

/**
  * \file test_operateur_minusDouble.cpp
  * \brief Programme de test de l'opérateur + entre un vecteur et un double
*/
int main() {

    Point v = Point(5,1,2);
    Point v2 = -v;
    assert(v2.getX() == -5);
    assert(v2.getY() == -1);
    assert(v2.getZ() == -2);

    return 0;
}
