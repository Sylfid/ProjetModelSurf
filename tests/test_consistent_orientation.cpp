#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>
#include "../src/cloud.h"
#include "../src//consistent_orientation.h"
#include "../src/load_off_file.h"

/**
  * \test test_consistent_orientation.cpp
  * \brief Programme qui teste que les étapes jusque "consistent plane orientation"
            fonctionne bien, en particulier teste la classe consistent_plane.
*/
int main(int argc, char** argv) {

    std::string filename;
    std::string name;
    if (argc == 3) {
        filename = argv[1];
        name = argv[2];
    } else if (argc == 2) {
        filename = argv[1];
        name = "test_consistent_orientation.off";
    } else {
        filename = "../../models/tetrahedron.off";
        if (!(boost::filesystem::exists(filename))) {
            filename = "../models/tetrahedron.off";
        }
        name = "test_cpo_tetrahedron.off";
    }

    std::cout << "lecture du fichier" << filename << std::endl;
    double debut_lecture = clock();
    // structure qui va stocker les points
    Cloud cloud(filename);
    double fin_lecture = clock();

    int K = 10;
    std::cout << "construction des pt ..." << std::endl;
    double debut_pt = clock();
    cloud.construct_tangent_planes(K);
    double fin_pt = clock();

    std::cout << "Consistent Plane Orientation ... " << std::endl << std::endl;
    int size = cloud.getSize();
    double debut_cpo = clock();
    orientation_algorithm(cloud.getPlanes(), size, K);
    double fin_cpo = clock();

    std::cout << "sauvegarde dans le fichier : " << name << std::endl;
    std::cout << "nombre de points : " << cloud.getSize() << std::endl;
    bool save_ok = save_cloud_OFF_file(name, cloud.getCloud());
    if (!(save_ok)) {
        return false;
    }

    std::cout << std::endl;
    std::cout << "===== TEMPS DEXECUTION : =====" << std::endl;
    std::cout << "o LECTURE DU .OFF : "
                << (fin_lecture-debut_lecture) / double(CLOCKS_PER_SEC)
                << "s" << std::endl;
    std::cout << "o CONSTRUCTION DES PT : "
                << (fin_pt-debut_pt) / double(CLOCKS_PER_SEC)
                << "s" << std::endl;
    std::cout << "o CONSISTENT ORIENTATION PLAN : "
                << (fin_cpo-debut_cpo) / double(CLOCKS_PER_SEC)
                << std::endl;
    std::cout << "===== TOTAL : "
                << (fin_pt-debut_lecture) / double(CLOCKS_PER_SEC)
                << "s =====" << std::endl;

    return EXIT_SUCCESS;

}
