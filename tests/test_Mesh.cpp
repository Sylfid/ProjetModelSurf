#include "../src/ImplicitFunction.h"
#include "../src/Mesh.cpp"
#include <iostream>
#include <string>
#include <boost/filesystem.hpp>
#include "../src/load_off_file.h"

/**
  * \test test_ImplicitFunction.cpp
  * \brief Programme qui teste la classe ImplicitFunction.cpp
*/
int main(int argc, char** argv) {

    int K = 3;
    double delta = 0.0;
    float isoValue = 0.0;

    std::string filename;
    std::string name;
    if (argc == 3) {
        filename = argv[1];
        name = argv[2];
    } else if (argc == 2) {
        filename = argv[1];
        name = "test_implfunct.off";
    } else {
        filename = "../../models/bunny.off";
        if (!(boost::filesystem::exists(filename))) {
            filename = "../models/bunny.off";
        }
        name = "test_implfunct.off";
    }

    std::cout << "OFF file : " << filename << std::endl;

    std::cout << "========== DEBUT : ==========" << std::endl << std::endl;
    double debut_sdf = clock();
    SignedDistanceFunction function(filename, K, delta);
    double fin_sdf = clock();

    std::cout << "Valeur de rho : " << function.getRho()
            << std::endl << std::endl;

    std::cout << "3. Construction du maillage en cours ..." << std::endl;
    double debut_mesh = clock();
    Mesh mesh;

    // on récupère les coordonnées min/max
    double minX = function.getMinX();
    double minY = function.getMinY();
    double minZ = function.getMinZ();
    double maxX = function.getMaxX();
    double maxY = function.getMaxY();
    double maxZ = function.getMaxZ();

    // on calcule la résolution de telle sorte à ce que les cubes soit de côté rho + delta
    unsigned int resX = (maxX-minX) / (function.getRho() + function.getDelta());
    unsigned int resY = (maxY-minY) / (function.getRho() + function.getDelta());
    unsigned int resZ = (maxZ-minZ) / (function.getRho() + function.getDelta());

    // marching tetrahedra
    mesh.CreateIsoSurface(function, isoValue, minX, maxX, minY, maxY, minZ, maxZ,
                            resX, resY, resZ);
    // mesh.Normalize();
    // mesh.ComputeNormals();
    //
    const char* name2 = "test_mesh.obj";
    mesh.write_obj(name2);
    double fin_mesh = clock();

    printf("-> Mesh créé avec %i point et %i faces\n", mesh.NbVertices(), mesh.NbFaces());
    std::cout << "Total : " << (fin_mesh-debut_mesh) / double(CLOCKS_PER_SEC) << std::endl;

    std::cout << "========== TOTAL : " << (fin_mesh-debut_sdf) / double(CLOCKS_PER_SEC)
                << "s ==========" << std::endl << std::endl;

    std::cout << "sauvegarde dans le fichier : " << name2 << std::endl;
    // std::cout << "nombre de points : " << function.getPointCloud().getCloud().size() << std::endl;
    // bool save_ok = save_cloud_OFF_file(name, function.getPointCloud().getCloud());
    // if (!(save_ok)) {
    //     return false;
    // }

    return EXIT_SUCCESS;

}
