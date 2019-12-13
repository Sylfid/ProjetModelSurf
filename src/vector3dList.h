#ifndef VECTOR3DLIST_H
#define VECTOR3DLIST_H

#include <string>
#include "vector3d.h"

class Vector3dList {

    private:
        Vector3d* points;
        int taille;

    public:
        Vector3dList();
        Vector3dList(int taille2);
        Vector3dList(Vector3dList const& taille2);
        Vector3dList(std::string file);
        ~Vector3dList();
        void display(std::ostream& str);
        Vector3dList getNbhd(int nbNb);
        Vector3d getVector3d(int i);
        void setVector3d(Vector3d point2, int i);
};

#endif // VECTOR3DLIST_H
