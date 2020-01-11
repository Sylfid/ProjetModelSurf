#include "../src/ImplicitFunction.h"
#include "../src/Mesh.h"
#include <iostream>
#include <string>
#include <boost/filesystem.hpp>
#include "../src/load_off_file.h"

/**
  * \test test_mesh.cpp
  * \brief Programme qui teste la classe mesh.cpp
*/
int main(int argc, char** argv) {

    int K = 4;
    double delta = 0;

    std::string filename;
    std::string name;
    if (argc == 3) {
        filename = argv[1];
        name = argv[2];
    } else if (argc == 2) {
        filename = argv[1];
        name = "test_implfunct.off";
    } else {
        filename = "../../models/buddha.off";
        if (!(boost::filesystem::exists(filename))) {
            filename = "../models/buddha.off";
        }
        name = "test_implfunct.off";
    }

    Mesh surfaceFinal;
    std::cout << "********** ETAPE DE LA RECONSTRUCTION : **********"
                << std::endl << std::endl;
    double debut = clock();
    SignedDistanceFunction functionFile(filename, K, delta);
    surfaceFinal.CreateIsoSurface2(functionFile, 100, 100, 100);
    double fin = clock();
    std::cout << "========== TOTAL : " << (fin-debut) / double(CLOCKS_PER_SEC)
                << "s ==========" << std::endl << std::endl;

    std::cout << "sauvegarde dans le fichier : " << name << std::endl;
    std::cout << "nombre de points : " << functionFile.getPointCloud().getCloud().size() << std::endl;
    surfaceFinal.write_off(name);

    return EXIT_SUCCESS;

}

