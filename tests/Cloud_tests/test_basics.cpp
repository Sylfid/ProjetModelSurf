#include "../../src/cloud.h"
#include "../../src/vector3d.h"
#include "../../src/load_off_file.h"
#include <boost/filesystem.hpp>
#include <assert.h>

/**
  * \test test_basics.cpp
  * \brief Programme qui teste les fonctions basiques de la classe cloud :
            constructeurs, getters, setters et autres méthodes classiques.
*/
int main () {

    Cloud cloud0;
    assert(cloud0.getSize() == 0);

    std::string filename0 = "../../../models/tetrahedron.off";
    if (!(boost::filesystem::exists(filename0))) {
        filename0 = "../models/tetrahedron.off";
    }

    cloud0.setPointCloud(filename0);
    bool save_ok = save_cloud_OFF_file("test_basics_tetrahedron.off", cloud0.getCloud());
    if (!(save_ok)) {
        return false;
    }
    int taille0 = cloud0.getCloud().size();
    assert(taille0 == cloud0.getSize());
    assert(cloud0.getSize() == 4);
    std::cout << "taille du nuage : " << cloud0.getSize() << std::endl;

    std::cout << "Test de displayCloud :" << std::endl;
    cloud0.displayCloud(std::cout);
    std::cout << "Test de displayPlanes :" << std::endl;
    cloud0.displayPlanes(std::cout);
    std::cout << "contructeur par défaut ok" << std::endl;

    for (std::vector<Vector3d>::iterator it = cloud0.getCloud().begin() ; it != cloud0.getCloud().end() ; ++it) {
        (*it).display(std::cout);
    }

    std::string filename = "../../../models/buddha.off";
    if (!(boost::filesystem::exists(filename))) {
        filename = "../models/buddha.off";
    }
    Cloud cloud(filename);
    bool save_ok1 = save_cloud_OFF_file("test_basics_buddha.off", cloud.getCloud());
    if (!(save_ok1)) {
        return false;
    }
    std::cout << "taille du nuage : " << cloud.getSize() << std::endl;
    int taille = cloud.getCloud().size();
    assert(taille == cloud.getSize());
    std::cout << "contructeur ok" << std::endl;

    return 0;
}
