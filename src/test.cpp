#include "../../../src/load_off_file.h"
#include "../../../src/vector3d.h"
#include <assert.h>
#include <sstream>
#include "Mesh.h"
#include "ImplicitFunction.h"


int main () {
    Mesh file("buddha.off");
    Mesh solution();
    vector<vec3> bbox(file.computeBB());
    MyImplicitFunction function("buddha.off");
    createIsoSurface(&solution, function, 0, bbox[0], bbox[1]);

    return 0;
}

