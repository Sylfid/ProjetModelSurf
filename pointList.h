#include <string>
#include "point.h"


class pointList{
    private:
        Point* points;
        int taille;
    public:
        pointList();
        pointList(int taille2);
        pointList(pointList const& taille2);
        pointList(std::string file);
        ~pointList();
};


