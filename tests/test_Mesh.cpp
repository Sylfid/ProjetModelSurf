#include "../src/ImplicitFunction.h"
#include "../src/Mesh.h"
#include <iostream>
#include <string>
#include <boost/filesystem.hpp>
#include "../src/load_off_file.h"

/**
  * \brief test_ImplicitFunction.cpp
 */
int main(int argc, char** argv) {

    /* PARSING DES ARGUMENTS */
    int K = 10;

    std::string filename;
    char* name;
    char* name2;

    if (argc == 4) {
        filename = argv[1];
        name = argv[2];
        name2 = argv[3];
    } else if (argc == 2) {
        filename = argv[1];
        name = "test_implfunct.off";
        name2 = "test_implfunct.obj";
    } else {
        filename = "../../models/cube_closed.off";
        if (!(boost::filesystem::exists(filename))) {
            filename = "../models/cube_closed.off";
        }
        name = "test_implfunct.off";
        name2 = "test_implfunct.obj";
    }

    std::cout << "OFF file : " << filename << std::endl;

    std::cout << "========== DEBUT : ==========" << std::endl << std::endl;
    double debut_sdf = clock();
<<<<<<< HEAD
    SignedDistanceFunction function(filename, K, INFINITY);
=======
    SignedDistanceFunction function(filename, K, delta);
    function.displaySignedDistanceFunction(std::cout);
>>>>>>> 8323ae03fdb23d7521a776e72a093ace6743371c
    double fin_sdf = clock();

    std::cout << "Valeur de rho + delta : " << function.getRhoPlusDelta()
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

    std::cout << minX << " " << maxX << "\n"
                << minY << " " << maxY << "\n"
                << minZ << " " << maxZ  << std::endl;

    // on calcule la résolution de telle sorte à ce que les cubes soit de côté rho + delta
    unsigned int resX = 10;
    unsigned int resY = 10;
    unsigned int resZ = 10;

    if (resX == 0 || resX == 1) resX=2;
    if (resY == 0 || resY == 1) resY=2;
    if (resZ == 0 || resZ == 1) resZ=2;
    float epsilon = 1e-7;

    std::cout << "resolution : " << resX << " " << resY << " " << resZ << std::endl;
    // marching tetrahedra
    mesh.CreateIsoSurface(function, 0.0, minX, maxX, minY, maxY, minZ, maxZ,
                            resX, resY, resZ);
    mesh.RemoveDouble(epsilon);
    // mesh.Normalize();
    // mesh.ComputeNormals();

    mesh.write_obj(name2);
    mesh.write_off(name);
    double fin_mesh = clock();

    printf("-> Mesh créé avec %i points et %i faces\n", mesh.NbVertices(), mesh.NbFaces());
    std::cout << "Total : " << (fin_mesh-debut_mesh) / double(CLOCKS_PER_SEC) << std::endl;

    std::cout << "========== TOTAL : " << (fin_mesh-debut_sdf) / double(CLOCKS_PER_SEC)
                << "s ==========" << std::endl << std::endl;

    std::cout << "sauvegarde dans le fichier off : " << name << std::endl;
    std::cout << "sauvegarde dans le fichier obj : " << name2 << std::endl;

    return EXIT_SUCCESS;
}
