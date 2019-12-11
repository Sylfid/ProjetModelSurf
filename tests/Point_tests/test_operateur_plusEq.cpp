#include "../../src/point.h"
#include <stdio.h>
#include <assert.h>

/**
  * \file test_operateur_plusEq.cpp
  * \brief Programme de test de l'opérateur +=
*/
int main() {

    Point v = Point(5,1,2);
    v += 2.7;
    assert(v.getX() == 7.7);
    assert(v.getY() == 3.7);
    assert(v.getZ() == 4.7);

    Point v1 = Point(1.3,3.2,0);
    v += v1;
    assert(v.getX() == 9.0);
    assert(v.getY() == 6.9);
    assert(v.getZ() == 4.7);

    return 0;
}
