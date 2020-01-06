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
    // // nécessaire ?
    // Vector3d u; /*!< un vecteur du plan */
    // Vector3d v; /*!< un autre vecteur du plan tel que (u,v,n) repère */

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
    // /**
    //   * \brief Constructeur de la classe.
    //   * \details Ce constructeur est appelé pour l'instanciation
    //   *             d'un plan avec son centre, sa normale et ses composantes
    //                 u et v.
    //   * \param center le centre du plan
    //   * \param normal la normale au plan
    //   * \param U la composante u
    //   * \param V la composante v
    // */
    // Plane(const Vector3d &center, const Vector3d &normal, const Vector3d &U, const Vector3d &V);
    /**
      * \brief Destructeur de la classe
    */
    ~Plane();

    /**
      * \brief Met à jour la normale au plan tangent
      * \param normal les nouvelles coordonées de la normale
    */
    void setNormal(const Vector3d &normal);
    /**
      * \brief Met à jour le centre du plan tangent
      * \param center les nouvelles coordonées du centre
    */
    void setCenter(const Vector3d &center);
    // /**
    //   * \brief Met à jour le centre du plan tangent
    //   * \param center les nouvelles coordonées du centre
    // */
    // void setU(const Vector3d &U);
    // /**
    //   * \brief Met à jour le centre du plan tangent
    //   * \param center les nouvelles coordonées du centre
    // */
    // void setV(const Vector3d &V);

    /**
      * \brief Récupère la normale
    */
    Vector3d getNormal() const;
    /**
      * \brief Récupère le centre
    */
    Vector3d getCenter() const;
    // /**
    //   * \brief Récupère la composante u
    // */
    // Vector3d getU() const;
    // /**
    //   * \brief Récupère la composante v
    // */
    // Vector3d getV() const;
    /**
      * \brief Affiche les composantes du plan
      * \details La méthode affichera, sur le pflux passé en paramètre, les infos
      * \param str le flux sur lequel s'affichera le résultat
    */
    void display(std::ostream& str);

};

#endif // PLANE_H
