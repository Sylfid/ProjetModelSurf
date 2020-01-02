#ifndef LOAD_OFF_FILE_H
#define LOAD_OFF_FILE_H

#include "vector3d.h"
#include <vector>

typedef std::numeric_limits< double > dbl;

/**
  * \brief Récupère les points 3d depuis le fichier OFF donnée en entrée
  * \details
  * \param filename le nom du fichier OFF
  * \param cloud l'objet vector qui stockent les points 3d qui sont des instances
  *             de Vector3d
*/
bool load_OFF_file(const std::string &filename, std::vector<Vector3d> &cloud);
/**
  * \brief Sauvegarde le nuage de points dans un fichier au format OFF
  * \param filename le nom du fichier OFF
  * \param cloud l'objet vector qui stockent les points 3d qui sont des instances
  *             de Vector3d
*/
bool save_cloud_OFF_file(const std::string &filename, std::vector<Vector3d> &cloud);

#endif // LOAD_OFF_FILE_H
