#include <exception>
#include <string>
#include <iostream>
#include "point.h"
#include <cstdlib>
#include <fstream>
#include <math.h>


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

double Point::getX() const{
    return x;
}

double Point::getY() const{
    return y;
}

double Point::getZ() const{
    return z;
}

Point operator-(Point const& a, Point const& b){

    Point resultat(a.getX() - b.getX(), a.getY() - b.getY(), a.getZ() - b.getZ());
    return resultat;

} 

Point operator+(Point const& a, Point const& b){

    Point resultat(a.getX() + b.getX(), a.getY() + b.getY(), a.getZ() + b.getZ());
    return resultat;

} 


double Point::getNorm(){
    return sqrt(x*x+y*y+z*z);
}

double Point::getDistance(Point point2){
    Point soustraction;
    soustraction = *this - point2;
    double resultat(soustraction.getNorm());
    return resultat;
}
