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


