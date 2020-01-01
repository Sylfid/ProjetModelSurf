#include "../src/load_off_file.h"

#include <iostream>
#include <limits>

/**
  * \test
  * \brief Programme qui teste la lecture d'un fichier off et qui vérifie que
  *         la récupération des données à bien été faite.
*/
int main (int argc, char** argv) {

    // fichier OFF à lire
    std::string filename = "../../models/test.off";

    // structure qui va stocker les points
    std::vector<Vector3d> cloud;

    // on récupère les points
    bool load_ok = load_OFF_file(filename, cloud);

    if (!(load_ok)) {
        return false;
    }

    std::string name = "test_test.off";
    bool save_ok = save_cloud_OFF_file(name, cloud);

    if (!(save_ok)) {
        return false;
    }

    return 0;
}
