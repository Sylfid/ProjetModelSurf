#include "load_off_file.h"
#include "vector3d.h"
#include <assert.h>
#include <sstream>
#include "Mesh.h"
#include "ImplicitFunction.h"


int main () {
    Mesh file();
    Mesh solution();
    std::vector<glm::vec3> bbox(file.computeBB());
    MyImplicitFunction function("../../models/buddha.off");
    // createIsoSurface(&solution, function, 0, bbox[0], bbox[1]);

    return 0;
}
