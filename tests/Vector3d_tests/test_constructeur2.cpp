#include "../../src/vector3d.h"
#include <stdio.h>
#include <assert.h>
#include <sstream>

/**
  * \test
  * \brief Programme de test de classe Vector3d
  *
  * \details Programme de test du constructeur par défaut
*/
int main() {

    Vector3d v = Vector3d();
    assert(v.getX() == 0.);
    assert(v.getY() == 0.);
    assert(v.getZ() == 0.);

    return 0;
}
