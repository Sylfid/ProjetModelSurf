#include "load_off_file.h"
#include <fstream>
#include <iostream>
#include <cassert>

bool load_OFF_file(const std::string &filename, std::vector<Vector3d> &cloud) {
    std::ifstream file(filename);
    std::string line;

    int n, a, b; // n : nombre de points à récupérer
    double x, y, z;

    if (!(file.is_open())) {
        throw std::runtime_error("Erreur : impossible d'ouvrir le fichier\n");
        return false;
    }

    file >> line;
    // on vérifie que le fichier est bien un fichier OFF
    if (line.compare("OFF")) {
        throw std::runtime_error("Format du fichier incorrect");
        file.close();
        return false;
    }
    // lecture de la deuxième ligne pour recupérer le nombre de points
    file >> n >> a >> b;
    // maj de la taille de la liste qui va contenir les points
    cloud.resize(n);

    // on récupère les coordonées des points 3d de notre nuage
    for (int i=0 ; i<n ; i++) {
        file >> x >> y >> z;
        cloud[i].set(x, y, z);
    }

    file.close();

    return true;
}

bool save_cloud_OFF_file(const std::string &filename, std::vector<Vector3d> &cloud) {
    std::ofstream file(filename);
    assert(file.is_open());

    file.precision(std::numeric_limits<double>::digits10+1);

    for (std::vector<Vector3d>::iterator it = cloud.begin() ; it != cloud.end() ; ++it) {
        file << it->getX() << " " << it->getY() << " " << it->getZ() << std::endl;
    }

    file.close();

    return true;
}
