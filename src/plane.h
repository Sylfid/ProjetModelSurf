#ifndef PLANE_H
#define PLANE_H

#include "vector3d.h"

/**
  * \brief Class représentant un plan défini par sa normale et son centre
*/
class Plane {

  private:
    Vector3d center; /*!< centre du plan */
    Vector3d normal; /*!< normal au plan */

  public:
    /**
      * \brief Constructeur par défaut de la classe.
      * \details Ce constructeur est appelé pour l'instanciation
      *             d'un plan. Il n'effectue rien.
    */
    Plane();
    /**
      * \brief Constructeur de la classe.
      * \details Ce constructeur est appelé pour l'instanciation
      *             d'un plan avec son centre et sa normale
      * \param center le centre du plan
      * \param normal la normale au plan
    */
    Plane(const Vector3d &center, const Vector3d &normal);
    /**
      * \brief Destructeur de la classe
    */
    ~Plane();

    void setNormal(const Vector3d &normal);
    void setCenter(const Vector3d &center);
    Vector3d getNormal() const;
    Vector3d getCenter() const;

    /**
      * \brief Affiche les composantes du plan
      * \details La méthode affichera, sur le pflux passé en paramètre, les infos
      * \param str le flux sur lequel s'affichera le résultat
    */
    void display(std::ostream& str);


};

#endif // PLANE_H
