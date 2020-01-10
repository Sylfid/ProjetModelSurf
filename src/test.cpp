#include "load_off_file.h"
#include "vector3d.h"
#include <assert.h>
#include <sstream>
#include "Mesh.h"
#include "ImplicitFunction.h"


int main () {
    Mesh file();
    Mesh solution();
    vector<vec3> bbox(file.computeBB());
    MyImplicitFunction function("../../models/buddha.off");
    std::ivec3 res(100.,100.,100.);
    createIsoSurface(&solution, function, 0, bbox[0], bbox[1], res);
    std::cout << displayMesh() ;
    

    return 0;
}
