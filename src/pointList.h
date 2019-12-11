#include <string>
#include "point.h"


class PointList {
  
    private:
        Point* points;
        int taille;

    public:
        PointList();
        PointList(int taille2);
        PointList(PointList const& taille2);
        PointList(std::string file);
        ~PointList();
        void display(std::ostream& str);
        PointList getNbhd(int nbNb);
        Point getPoint(int i);
        void setPoint(Point point2, int i);
};
