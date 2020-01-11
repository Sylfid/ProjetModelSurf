#include "../src/ImplicitFunction.h"
#include <iostream>
#include <string>
#include <boost/filesystem.hpp>
#include "../src/load_off_file.h"

/**
  * \test test_ImplicitFunction.cpp
  * \brief Programme qui teste la classe ImplicitFunction.cpp
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

    std::cout << "===== TEMPS DEXECUTION : =====" << std::endl << std::endl;
    double debut = clock();
    SignedDistanceFunction SignedDistanceFunction(filename, K, delta);
    double fin = clock();
    std::cout << "===== TOTAL : " << (fin-debut) / double(CLOCKS_PER_SEC)
                << "s =====" << std::endl;

    std::cout << "sauvegarde dans le fichier : " << name << std::endl;
    std::cout << "nombre de points : " << SignedDistanceFunction.getPointCloud().getCloud().size() << std::endl;
    bool save_ok = save_cloud_OFF_file(name, SignedDistanceFunction.getPointCloud().getCloud());
    if (!(save_ok)) {
        return false;
    }

    return EXIT_SUCCESS;

}
