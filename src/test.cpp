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
    SignedDistanceFunction function2("../../models/buddha.off",5,0.1);
    std::ivec3 res(100.,100.,100.);
    createIsoSurface(&solution, function2, 0, bbox[0], bbox[1], res);
    std::cout << displayMesh() ;
    

    return 0;
}
