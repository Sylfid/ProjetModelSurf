#include <exception>
#include <string>
#include <iostream>
#include "point.h"
#include <cstdlib>
#include <fstream>


Point::Point(){
    x = 0.;
    y = 0.;
    z = 0.;
}

Point::Point(double X, double Y, double Z){
    x=X;
    y=Y;
    z=Z;
}

void Point::display(std::ostream& str){
    str << x << " " << y << " " << z << "\n";
}

void Point::set(double X, double Y, double Z){
    x= X;
    y=Y;
    z=Z;
}

void Point::setX(double X){
    x= X;
}

void Point::setY(double Y){
    y= Y;
}

void Point::setZ(double Z){
    z= Z;
}
