#include <string>

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
      * \brief Renvoie le résultat du produit scalaire avec un autre point
      * \param P le point avec lequel faire le produit scalaire
    */
    double getScalarProduct(const Point &P);
    /**
      * \brief Renvoie la norme 2 du point
    */
    double getNorm();
    /**
      * \details Ce Constructeur est appelé pour l'instanciation
      *             d'un objet Dvector. Créé un vecteur de taille 1 de valeur 0.
      * \brief Renvoie la distance entre le point et celui donné en argument
      * \param P lepoint par rapport auquel on calcule la distance
    */
    double getDistance(const Point &P);
    /**
      * \brief operateur d'affectation de la classe
      * \details L'opérateur = permet de réaliser l'opération suivante :
                    Point v; v = Point(1,2,3);
      * \param P l'objet Point qu'il faut affecté à l'objet Point
    */
    Point& operator=(const Point &P);
};

Point operator-(Point const& a, Point const& b);
Point operator+(Point const& a, Point const& b);

/**
  * \brief opérateur de flux qui affiche le contenu du vecteur donné
            en paramètre sur le flux donné en paramètre
  * \param str le flux
  * \param V le vecteur dont il faut affiche le contenu
  * \return affiche le contenu du vecteur sur le flux
*/
std::ostream& operator<<(std::ostream &str, const Point &P);
