#include <string>
#include <iostream>
#include "point.h"
#include <cstdlib>
#include <fstream>
#include <assert.h>
#include <math.h>

Point::Point(){
    x = 0.;
    y = 0.;
    z = 0.;
}

Point::Point(double x, double y, double z):x(x), y(y), z(z){
  // rien à faire
}

Point::Point(const Point &P):x(P.x), y(P.y), z(P.z) {
  // rien à faire
}

Point::~Point() {
  // rien à faire
}

void Point::display(std::ostream& str){
    str << x << "\t" << y << "\t" << z << "\n";
}

double Point::getScalarProduct(const Point &P) {
  return x*P.x + y*P.y + z*P.z;
}

Point Point::crossProduct(const Point &P) {
  Point CP;
  Point V = *this;
  CP.x = V.y*P.z - V.z*P.y;
  CP.y = V.z*P.z - V.x*P.z;
  CP.z = V.x*P.y - V.y*P.x;
  return CP;
}

double Point::getNorm(){
    return sqrt(x*x+y*y+z*z);
}

double Point::getDistanceTo(const Point &P){
    Point soustraction;
    soustraction = *this - P;
    double distance(soustraction.getNorm());
    return distance;
}

void Point::set(double X, double Y, double Z) {
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

void Point::normalize() {
    double norm = (*this).getNorm();
    x /= norm;
    y /= norm;
    z /= norm;
}

// OPERATEURS

Point& Point::operator+=(const double d) {
    x += d;
    y += d;
    z += d;
    return *this;
}

Point& Point::operator-=(const double d) {
    return *this += -d;
}

Point& Point::operator*=(const double d) {
    x *= d;
    y *= d;
    z *= d;
    return *this;
}

Point& Point::operator/=(const double d) {
    if (d == 0.0) {
        throw std::domain_error("Division par zéro.");
    }
    Point &P = *this;
    P.x /= d;
    P.y /= d;
    P.z /= d;
    return P;
}

Point& Point::operator+=(const Point &P) {
    x += P.x;
    y += P.y;
    z += P.z;
    return *this;
}

Point& Point::operator-=(const Point &P) {
    Point &V = *this;
    V += -1*P;
    return V;
}

Point& Point::operator=(const Point &P) {
  Point &V = *this;
  V.x = P.x;
  V.y = P.y;
  V.z = P.z;
  return V;
}

// -----------------------------------------------------------

// OPERATEURS

Point operator-(const Point &P) {
  Point V(P);
  V *= -1;
  return V;
}

Point operator+(const Point &P1, const Point &P2) {
    Point P(P1);
    P += P2;
    return P;
}

Point operator-(const Point &P1, const Point &P2){
    Point P(P1);
    P -= P2;
    return P;
}

Point operator*(const double d, const Point &P) {
  Point V(P);
  V *= d;
  return V;
}

Point operator/(const Point &P, const double d) {
  if (d == 0.0) {
    throw std::domain_error("Division par zéro.");
  }
  Point V = (1/d) * P;
  return V;
}

std::ostream& operator<<(std::ostream &str, const Point &P) {
  str << P.getX() << "\t" << P.getY() << "\t" << P.getZ() << "\n";
  return str;
}
