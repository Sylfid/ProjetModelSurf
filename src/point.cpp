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
    x = X;
    y = Y;
    z = Z;
}

Point::~Point() {
  // rien à faire
}

void Point::display(std::ostream& str){
    str << x << "\t" << y << "\t" << z << "\n";
}

void Point::set(double X, double Y, double Z){
    x = X;
    y = Y;
    z = Z;
}

void Point::setX(double X){
    x = X;
}

void Point::setY(double Y){
    y = Y;
}

void Point::setZ(double Z){
    z = Z;
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

Point& Point::operator=(const Point &P) {
  x = P.getX();
  y = P.getY();
  z = P.getZ();
  return *this;
}

Point operator-(Point const& a, Point const& b){
    Point P(a.getX() - b.getX(), a.getY() - b.getY(), a.getZ() - b.getZ());
    return P;
}

Point operator+(Point const& a, Point const& b){
    Point P(a.getX() + b.getX(), a.getY() + b.getY(), a.getZ() + b.getZ());
    return P;
}

double Point::getScalarProduct(const Point &P) {
  double scalarProduct = x*P.getX() + y*P.getY() + z*P.getZ();
  return scalarProduct;
}

double Point::getNorm(){
    return sqrt(x*x+y*y+z*z);
}

double Point::getDistance(const Point &P){
    Point soustraction;
    soustraction = *this - P;
    double distance(soustraction.getNorm());
    return distance;
}

std::ostream& operator<<(std::ostream &str, const Point &P) {
  str << P.getX() << "\t" << P.getY() << "\t" << P.getZ() << "\n";
  return str;
}
