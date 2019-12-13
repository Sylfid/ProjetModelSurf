#include "../../src/vector3d.h"
#include <stdio.h>
#include <sstream>
#include <assert.h>

/**
  * \file test_operateur_oflux.cpp
  * \brief Programme de test de l'opérateur de flux
*/
int main() {

    Vector3d v = Vector3d(3.5,0,2.5);
    std::stringstream str;
    v.display(str);

    std::stringstream str1;
    str1 << v;
    assert(str.str() == str1.str());

    return 0;
}
