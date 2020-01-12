#include "src/ImplicitFunction.h"
#include "src/Mesh.h"
#include <iostream>
#include <string>
#include <boost/filesystem.hpp>
#include "src/load_off_file.h"

/**
  * \brief Fonction de reconstruction basée sur l'algorithme décrit dans l'article
            "Surface Reconstruction from Unorganised Points" (Hoppe92 et al. SIGGRAPH)
*/
int main(int argc, char** argv) {

    if (argc < 4) {
        std::cout << "Usage : \n"
                << argv[0] << " off_input obj_ouput K [density] [noise]" << std::endl;
        return EXIT_FAILURE;
    }

    /* début initlisation des paramètres */
    std::string filename = argv[1];
    char* name = argv[2];
    int K = std::stoi(argv[3]);

    double density = INFINITY;
    double noise = INFINITY;
    if (argc > 4) density = std::stod(argv[4]);
    if (argc > 5) noise = std::stod(argv[5]);

    double rho_plus_delta = density + noise;
    /* fin initlisation des paramètres */

    std::cout <<"\nParamètres d'entrées : " << std::endl;
    std::cout << "Fichier d'entrée : \t" << filename << std::endl;
    std::cout << "Fichier de sortie : \t" << name << std::endl;
    std::cout << "Nombre de voisins : \tK = " << K << std::endl;
    std::cout << "Paramètre de bruit : \trho + delta = " << rho_plus_delta
                << std::endl << std::endl;

    std::cout << "**************** DEBUT ****************" << std::endl << std::endl;
    double debut_sdf = clock();
    SignedDistanceFunction function(filename, K, rho_plus_delta);
    double fin_sdf = clock();

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
    unsigned int resX = 10;
    unsigned int resY = 10;
    unsigned int resZ = 10;

    if (resX == 0 || resX == 1) resX=2;
    if (resY == 0 || resY == 1) resY=2;
    if (resZ == 0 || resZ == 1) resZ=2;
    float epsilon = 1e-7;

    std::cout << "\tresolution (x y z) : " << resX << " " << resY << " " << resZ << std::endl;
    // marching tetrahedra
    mesh.CreateIsoSurface(function, 0.0, minX, maxX, minY, maxY, minZ, maxZ,
                            resX, resY, resZ);
    mesh.RemoveDouble(epsilon);

    double fin_mesh = clock();
    mesh.write_obj(name);

    std::cout << "o MARCHING TETRAHEDRA : " << (fin_mesh-debut_mesh) / double(CLOCKS_PER_SEC) << std::endl;

    std::cout << "========== TOTAL : " << (fin_mesh-debut_sdf) / double(CLOCKS_PER_SEC)
                << "s ==========" << std::endl << std::endl;

    std::cout << "**************** FIN ****************"
                << std::endl << std::endl;

    std::cout << "---> Maillage créé avec " << mesh.NbVertices() << " sommets et "
                << mesh.NbFaces() << " faces\n" << std::endl;

    std::cout << "Maillage sauvegardé dans le fichier : " << name << std::endl;

    return EXIT_SUCCESS;
}
