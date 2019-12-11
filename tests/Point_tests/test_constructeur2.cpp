#include "../../src/point.h"
#include <stdio.h>
#include <assert.h>
#include <sstream>

/**
  * \test
  * \brief Programme de test de classe Point
  *
  * \details Programme de test du constructeur par défaut
*/
int main() {

    Point v = Point();
    assert(v.getX() == 0.);
    assert(v.getY() == 0.);
    assert(v.getZ() == 0.);

    return 0;
}
