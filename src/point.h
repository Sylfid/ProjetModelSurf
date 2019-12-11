#ifndef POINT_H
#define POINT_H

#include <string>
#include <iostream>

/**
  * \brief Classe représentant un point 3D de coordonées (x,y,z)
*/
class Point {

  private:
    double x; /*!< abscisse du point 3D */
    double y; /*!< ordonnée du point 3D */
    double z; /*!< côte du point 3D */

  public:
    /**
      * \brief Constructeur par défaut de la classe.
      * \details Ce Constructeur est appelé pour l'instanciation
      *             d'un point 3D de coordonées (0,0,0)
    */
    Point();
    /**
      * \brief Constructeur de la classe.
      * \details Ce constructeur crée un point 3D
      * \param x abscisse
      * \param y ordonnée
      * \param z côte
    */
    Point(double x, double y, double z);
    /**
      * \brief Constructeur par recopie de la classe.
      * \param P le point à recopier.
    */
    Point(const Point &P);
    /**
      * \brief Destructeur de la classe
    */
    ~Point();
    /**
      * \brief Affiche les coordonnées du point 3D.
      * \details La méthode affichera, sur le flux passé en paramètre, le point
      * \param str le flux sur lequel s'affichera le résultat
    */
    void display(std::ostream& str);
    void set(double x, double y, double z);
    void setX(double x);
    void setY(double y);
    void setZ(double z);
    double getX() const;
    double getY() const;
    double getZ() const;

    /**
      * \brief Normalise le vecteur
    */
    void normalize();

    /**
      * \brief Renvoie le résultat du produit scalaire avec un autre point
      * \param P le point avec lequel faire le produit scalaire
    */
    double getScalarProduct(const Point &P);
    /**
      * \brief Renvoie le résultat du produit vectoriel avec un autre point
      * \param P le point avec lequel faire le produit scalaire
    */
    Point crossProduct(const Point &P);
    /**
      * \brief Renvoie la norme 2 du point
    */
    double getNorm();
    /**
      * \details Ce Constructeur est appelé pour l'instanciation
      *             d'un objet Point. Créé un vecteur de taille 1 de valeur 0.
      * \brief Renvoie la distance entre le point et celui donné en argument
      * \param P lepoint par rapport auquel on calcule la distance
    */
    double getDistanceTo(const Point &P);

    // OPERATEURS

    /**
      * \brief opérateur +=
      * \details L'opérateur += permet de réaliser l'opération suivante :
                    v += b qui est équivalente à : v(i) += b pour tout
                    élément de v avec v un point, b un double
      * \param d le nombre b de l'opération
      * \return le résultat de l'opération v += d
    */
    Point& operator+=(const double d);
    /**
    * \brief opérateur -=
    * \details L'opérateur -= permet de réaliser l'opération suivante :
                    v -= b qui est équivalente à : v(i) -= b pour tout
                    élément de v avec v un Point, b un double
    * \param d le nombre b de l'opération
    * \return le résultat de l'opération v -= d
    */
    Point& operator-=(const double d);
    /**
    * \brief opérateur *=
    * \details L'opérateur *= permet de réaliser l'opération suivante :
                v *= b qui est équivalente à : v(i) *= b pour tout
                élément de v avec v un Point, b un double
    * \param d le nombre b de l'opération
    * \return le résultat de l'opération v *= d
    */
    Point& operator*=(const double d);
    /**
    * \brief opérateur /=
    * \details L'opérateur /= permet de réaliser l'opération suivante :
                  v /= b qui est équivalente à : v(i) /= b pour tout
                  élément de v avec v un Point, b un double
    * \param d le nombre b de l'opération
    * \return le résultat de l'opération v /= d
    */
    Point& operator/=(const double d);
    /**
      * \brief opérateur +=
      * \details L'opérateur += permet de réaliser l'opération suivante :
                    v1 += v2 qui est équivalente à : v1 = v1 + v2
                    avec v1 et v2 des Point.
                    Cette opérateur réalise l'addition terme à terme.
      * \param V l'objet Point v2
      * \return le résultat de l'opération v2 += V
    */
    Point& operator+=(const Point &V);
    /**
      * \brief opérateur -=
      * \details L'opérateur += permet de réaliser l'opération suivante :
                    v1 -= v2 qui est équivalente à : v1 = v1 - v2
                    avec v1 et v2 des Point.
                    Cette opérateur réalise la soustraction terme à terme.
      * \param V l'objet Point v2
      * \return le résultat de l'opération v1 += V
    */
    Point& operator-=(const Point &V);

    /**
      * \brief operateur d'affectation de la classe
      * \details L'opérateur = permet de réaliser l'opération suivante :
                    Point v; v = Point(1,2,3);
      * \param P l'objet Point qu'il faut affecté à l'objet Point
    */
    Point& operator=(const Point &P);

};
/**
  * \brief opérateur unaire -
  * \details L'opérateur applique l'opérateur uniaire - à tous les éléments
                du vecteur.
  * \return le résultat de l'opération -v
*/
Point operator-(const Point &P);
/**
  * \brief opérateur de soustraction entre deux vecteurs
  * \details L'opérateur réaliser l'addition binaire entre deux vecteurs.
  * \param V1 vecteur à gauche de l'opérande -
    \param V2 le veteur à droite de l'opérande -
  * \return le résultat de l'opération P1 - P2
*/
Point operator-(const Point &P1, const Point &P2);
/**
  * \brief opérateur d'addition entre deux vecteurs
  * \details L'opérateur réaliser l'addition binaire entre deux vecteurs.
  * \param P1 vecteur à gauche de l'opérande +
    \param P2 le veteur à droite de l'opérande +
  * \return le résultat de l'opération P1 + P2
*/
Point operator+(const Point &P1, const Point &P2);
/**
  * \brief opérateur * qui multiplie un point et un scalaire
  * \details L'opérateur réaliser l'addition suivante :
                d.P
                avec d le scalaire et P le point
  * \param P le point p
  * \param d le double d
  * \return le résultat de l'opération d.P;
*/
Point operator*(const double d, const Point &P);

/**
  * \brief opérateur de flux qui affiche le contenu du vecteur donné
            en paramètre sur le flux donné en paramètre
  * \param str le flux
  * \param V le vecteur dont il faut affiche le contenu
  * \return affiche le contenu du vecteur sur le flux
*/
std::ostream& operator<<(std::ostream &str, const Point &P);

#endif // POINT_H
