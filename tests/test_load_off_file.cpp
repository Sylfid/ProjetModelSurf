#include "../src/load_off_file.h"
#include <limits>
#include <boost/filesystem.hpp>

/**
  * \test
  * \brief Programme qui teste la lecture d'un fichier off et qui vérifie que
  *         la récupération des données à bien été faite.
*/
int main (int argc, char** argv) {

    std::string filename;
    std::string name;
    if (argc == 3) {
        filename = argv[1];
        name = argv[2];
    } else if (argc == 2) {
        filename = argv[1];
        name = "test_load_off.off";
    } else {
        filename = "../../models/buddha.off";
        if (!(boost::filesystem::exists(filename))) {
            filename = "../models/buddha.off";
        }
        name = "test_buddha.off";
    }

    std::cout << "lecture du fichier : " << filename << std::endl;
    // structure qui va stocker les points
    std::vector<Vector3d> cloud;

    // on récupère les points
    bool load_ok = load_OFF_file(filename, cloud);
    if (!(load_ok)) {
        return false;
    }

    std::cout << "sauvegarde dans le fichier : " << name << std::endl;
    bool save_ok = save_cloud_OFF_file(name, cloud);
    if (!(save_ok)) {
        return false;
    }

    return EXIT_SUCCESS;
}
