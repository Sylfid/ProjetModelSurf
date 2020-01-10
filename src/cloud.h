#ifndef CLOUD_H
#define CLOUD_H

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "vector3d.h"
#include "plane.h"
#include <vector>
#include <Eigen/Eigen>
#include <Eigen/Dense>

/**
  * \brief Représente un nuage de points 3d
  * \details Le plan d'indice i de l'attribut planes est le plan tangent
                associé un point d'indice de l'attribut cloud. Le nombre de
                points de cloud est égale au nombre de plan dans planes.
*/
class Cloud {

    private:
        //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;  /*!< liste des points du nuage */
        std::vector<Vector3d> cloud; /*!< liste des points du nuage */
        std::vector<Plane> planes; /*!< liste des plans tangents */
        int size; /*!< nombre de points à étudier (=le nombre de plans) */
        double rho; /*!< paramètre rho qu'on initialise au moment de la construction des plans tangents
                        (sa valeur est nulle à l'appel du constructeur)*/
        double delta; /*!< paramètre de bruit delta de l'article */

    public:
        /**
          * \brief Constructeur par défaut de la classe.
          * \details Ne fait rien.
        */
        Cloud();
        /**
          * \brief Constructeur de la classe.
          * \details Ce constructeur initialise l'attribut cloud en récupérant
                        le nuage de points contenu dans le fichier OFF donnée
                        en paramètre.
          * \param filename le nom (+le chemin) du fichier OFF dont il faut
                        recupérer le nuage de points.
          * \param d la valeur du paramètre delta
        */
        Cloud(const std::string &filename, const double d=0.0);
        /**
          * \brief Destructeur de la classe
        */
        ~Cloud();
        /**
          * \brief Mets à jour les attributs cloud (nuage de points) et size
                    à partir d'un fichier OFF.
          * \param filename nom du fichier OFF contenant le nuage à récupérer
        */
        void setPointCloud(const std::string &filename);
        /**
          * \brief Affiche la liste des points sur le flux donnée en paramètre.
          * \param str le flux de sortie surlequel afficher la liste.
        */
        void displayCloud(std::ostream& str) const;
        /**
          * \brief Affiche la liste des plans tangents sur le flux donnée en paramètre.
          * \param str le flux de sortie surlequel afficher la liste.
        */
        void displayPlanes(std::ostream& str) const;
        /**
          * \brief Renvoie la taille du vecteur
        */
        int getSize() const;
        /**
          * \brief Renvoie la valeur de rho (le plus petit rayon r tel que la
                    sphére de rayon r et de rayon un point du nuage contient au
                    moins un autre point du nuage);
        */
        double getRho() const;
        /**
          * \brief Renvoie la valeur de delta
        */
        double getDelta() const;
        /**
          * \brief Met à jour la valeur de delta
        */
        void setDelta(cont double d);
        /**
        * \brief Renvoie l'attribut cloud (le nuage de points)
        */
        std::vector<Vector3d> &getCloud();
        /**
        * \brief Renvoie l'attribut cloud (le nuage de points)
        */
        std::vector<Vector3d> getCloud() const;
        /**
        * \brief Renvoie le point d'indice i du nuage de points
        */
        Vector3d getCloudPrecise(const int i) const;
        /**
        * \brief Renvoie le point d'indice i du nuage de points
        */
        Vector3d &getCloudPrecise(const int i);
        /**
          * \brief Renvoie l'attribut planes (la liste des plans tangents)
        */
        std::vector<Plane> &getPlanes();
        /**
          * \brief Renvoie l'attribut planes (la liste des plans tangents)
        */
        std::vector<Plane> getPlanes() const;
        /**
          * \brief Renvoie le plan d'indice i de la liste Plane
        */
        Plane getPlanePrecise(const int i) const;
        /**
          * \brief Renvoie le plan d'indice i de la liste Plane
        */
        Plane &getPlanePrecise(const int i);
        /**
          * \brief Calcule tous les plans tangents.
          * \param K le nombre de voisins pour le calcul des kNNbhd
        */
        void construct_tangent_planes(const int K);
};

/**
  * \brief Calcule le centroïde d'un ensemble de points.
  * \details Ce centroïde est calculé comme étant le barycentre de l'ensemble
                de points.
  * \param nbhd l'ensemble de points dont il faut calculer le barycentre
  * \return le centroïde qui est de type Vector3d
*/
Vector3d compute_3d_centroid(std::vector<Vector3d> nbhd);

/**
  * \brief Calcule la normale d'un plan associé à un ensemble de points.
  * \details La normal est le 3e vecteur propre de la matrice de covariance de
                la PCA de l'ensemble de points.
  * \param nbhd l'ensemble de points dont il faut calculer le barycentre
  * \param o le centroïde du plan P
  * \param P le plan P dont il faut calculer la normale
*/
void compute_normal(std::vector<Vector3d> nbhd, Vector3d o, Plane &P);

#endif // CLOUD_H
