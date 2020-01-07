#ifndef VECTOR3D_H
#define VECTOR3D_H

#include <string>
#include <iostream>
#include <limits>

/**
  * \brief Classe représentant un vecteur 3D de coordonées (x,y,z)
*/
class Vector3d {

private:
    double x; /*!< abscisse du vecteur 3D */
    double y; /*!< ordonnée du vecteur 3D */
    double z; /*!< côte du vecteur 3D */

  public:
    /**
      * \brief Constructeur par défaut de la classe.
      * \details Ce Constructeur est appelé pour l'instanciation
      *             d'un vecteur 3D de coordonées (0,0,0)
    */
    Vector3d();
    /**
      * \brief Constructeur de la classe.
      * \details Ce constructeur crée un vecteur 3D
      * \param x abscisse
      * \param y ordonnée
      * \param z côte
    */
    Vector3d(double x, double y, double z);
    /**
      * \brief Constructeur par recopie de la classe.
      * \param P le vecteur à recopier.
    */
    Vector3d(const Vector3d &P);
    /**
      * \brief Destructeur de la classe
    */
    ~Vector3d();
    /**
      * \brief Affiche les coordonnées du vecteur 3D.
      * \details La méthode affichera, sur le flux passé en paramètre, le vecteur
      * \param str le flux sur lequel s'affichera le résultat
    */
    void display(std::ostream& str);
    /**
      * \brief Mets à jour les coordonnées du vecteur.
      * \param x coordonée x
      * \param y coordonée y
      * \param z coorodnée z
    */
    void set(double x, double y, double z);
    /**
      * \brief Mets à jour la coordonnée x du vecteur.
      * \param x coordonée x
    */
    void setX(double x);
    /**
      * \brief Mets à jour la coordonnée y du vecteur.
      * \param y coordonée y
    */
    void setY(double y);
    /**
      * \brief Mets à jour la coordonnée z du vecteur.
      * \param z coorodnée z
    */
    void setZ(double z);
    /**
      * \brief Recupére la cordonnée x du vecteur.
    */
    double getX() const;
    /**
      * \brief Recupére la cordonnée y du vecteur.
    */
    double getY() const;
    /**
      * \brief Recupére la cordonnée z du vecteur.
    */
    double getZ() const;

    /**
      * \brief Normalise le vecteur
    */
    void normalize();

    /**
      * \brief Renvoie le résultat du produit scalaire avec un autre vecteur
      * \param P le vecteur avec lequel faire le produit scalaire
    */
    double getScalarProduct(const Vector3d &P) const;
    /**
      * \brief Renvoie le résultat du produit vectoriel avec un autre vecteur
      * \param P le vecteur avec lequel faire le produit scalaire
    */
    Vector3d crossProduct(const Vector3d &P) const;
    /**
      * \brief Renvoie la norme 2 du vecteur
    */
    double getNorm();
    /**
      * \details Ce Constructeur est appelé pour l'instanciation
      *             d'un objet Vector3d. Créé un vecteur de taille 1 de valeur 0.
      * \brief Renvoie la distance entre le vecteur et celui donné en argument
      * \param P levecteur par rapport auquel on calcule la distance
    */
    double getDistanceTo(const Vector3d &P);

    // OPERATEURS
    /**
          * \brief opérateur d'accès à un élément du vecteur à l'aide de []
          * \param i un entier compris entre 0 et la taille du vecteur 2
          * \return Renvoie une référence :
                        le ieme élément du vecteur lorsque cela a un sens
        */
    double operator[](int i) const;
    /**
          * \brief opérateur d'accès à un élément du vecteur à l'aide de ()
          * \param i un entier compris entre 0 et la taille du vecteur 2
          * \return Renvoie une référence :
                        le ieme élément du vecteur lorsque cela a un sens
        */
    double operator()(int i) const;
    /**
      * \brief opérateur +=
      * \details L'opérateur += permet de réaliser l'opération suivante :
                    v += b qui est équivalente à : v(i) += b pour tout
                    élément de v avec v un vecteur, b un double
      * \param d le nombre b de l'opération
      * \return le résultat de l'opération v += d
    */
    Vector3d& operator+=(const double d);
    /**
    * \brief opérateur -=
    * \details L'opérateur -= permet de réaliser l'opération suivante :
                    v -= b qui est équivalente à : v(i) -= b pour tout
                    élément de v avec v un Vector3d, b un double
    * \param d le nombre b de l'opération
    * \return le résultat de l'opération v -= d
    */
    Vector3d& operator-=(const double d);
    /**
    * \brief opérateur *=
    * \details L'opérateur *= permet de réaliser l'opération suivante :
                v *= b qui est équivalente à : v(i) *= b pour tout
                élément de v avec v un Vector3d, b un double
    * \param d le nombre b de l'opération
    * \return le résultat de l'opération v *= d
    */
    Vector3d& operator*=(const double d);
    /**
    * \brief opérateur /=
    * \details L'opérateur /= permet de réaliser l'opération suivante :
                  v /= b qui est équivalente à : v(i) /= b pour tout
                  élément de v avec v un Vector3d, b un double
    * \param d le nombre b de l'opération
    * \return le résultat de l'opération v /= d
    */
    Vector3d& operator/=(const double d);
    /**
      * \brief opérateur +=
      * \details L'opérateur += permet de réaliser l'opération suivante :
                    v1 += v2 qui est équivalente à : v1 = v1 + v2
                    avec v1 et v2 des Vector3d.
                    Cette opérateur réalise l'addition terme à terme.
      * \param V l'objet Vector3d v2
      * \return le résultat de l'opération v2 += V
    */
    Vector3d& operator+=(const Vector3d &V);
    /**
      * \brief opérateur -=
      * \details L'opérateur += permet de réaliser l'opération suivante :
                    v1 -= v2 qui est équivalente à : v1 = v1 - v2
                    avec v1 et v2 des Vector3d.
                    Cette opérateur réalise la soustraction terme à terme.
      * \param V l'objet Vector3d v2
      * \return le résultat de l'opération v1 += V
    */
    Vector3d& operator-=(const Vector3d &V);

    /**
      * \brief operateur d'affectation de la classe
      * \details L'opérateur = permet de réaliser l'opération suivante :
                    Vector3d v; v = Vector3d(1,2,3);
      * \param P l'objet Vector3d qu'il faut affecté à l'objet Vector3d
    */
    Vector3d& operator=(const Vector3d &P);

};
/**
  * \brief opérateur unaire -
  * \details L'opérateur applique l'opérateur uniaire - à tous les éléments
                du vecteur.
  * \return le résultat de l'opération -v
*/
Vector3d operator-(const Vector3d &P);
/**
  * \brief opérateur de soustraction entre deux vecteurs
  * \details L'opérateur réaliser l'addition binaire entre deux vecteurs.
  * \param V1 vecteur à gauche de l'opérande -
    \param V2 le veteur à droite de l'opérande -
  * \return le résultat de l'opération P1 - P2
*/
Vector3d operator-(const Vector3d &P1, const Vector3d &P2);
/**
  * \brief opérateur d'addition entre deux vecteurs
  * \details L'opérateur réaliser l'addition binaire entre deux vecteurs.
  * \param P1 vecteur à gauche de l'opérande +
    \param P2 le veteur à droite de l'opérande +
  * \return le résultat de l'opération P1 + P2
*/
Vector3d operator+(const Vector3d &P1, const Vector3d &P2);
/**
  * \brief opérateur * qui multiplie un vecteur et un scalaire
  * \details L'opérateur réaliser l'addition suivante :
                d.P
                avec d le scalaire et P le vecteur
  * \param P le vecteur p
  * \param d le double d
  * \return le résultat de l'opération d.P;
*/
Vector3d operator*(const double d, const Vector3d &P);

/**
  * \brief opérateur de flux qui affiche le contenu du vecteur donné
            en paramètre sur le flux donné en paramètre
  * \param str le flux
  * \param V le vecteur dont il faut affiche le contenu
  * \return affiche le contenu du vecteur sur le flux
*/
std::ostream& operator<<(std::ostream &str, const Vector3d &P);
/**
  * \brief Renvoie le résultat du produit scalaire entre deux vecteurs
  * \param P le vecteur avec lequel faire le produit scalaire
*/
double dot(const Vector3d &P, const Vector3d &V);

#endif // VECTOR3D_H
