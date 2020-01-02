#include "vector3d.h"
#include <cstdlib>
#include <fstream>
#include <assert.h>
#include <math.h>

Vector3d::Vector3d(){
    x = 0.;
    y = 0.;
    z = 0.;
}

Vector3d::Vector3d(double x, double y, double z):x(x), y(y), z(z){
  // rien à faire
}

Vector3d::Vector3d(const Vector3d &P):x(P.x), y(P.y), z(P.z) {
  // rien à faire
}

Vector3d::~Vector3d() {
  // rien à faire
}

void Vector3d::display(std::ostream& str){
    str.precision(std::numeric_limits<double>::digits10+1);
    str << x << "\t" << y << "\t" << z << "\n";
}

double Vector3d::getScalarProduct(const Vector3d &P) {
  return x*P.x + y*P.y + z*P.z;
}

Vector3d Vector3d::crossProduct(const Vector3d &P) {
  Vector3d CP;
  Vector3d V = *this;
  CP.x = V.y*P.z - V.z*P.y;
  CP.y = V.z*P.z - V.x*P.z;
  CP.z = V.x*P.y - V.y*P.x;
  return CP;
}

double Vector3d::getNorm(){
    return sqrt(x*x+y*y+z*z);
}

double Vector3d::getDistanceTo(const Vector3d &P){
    Vector3d soustraction;
    soustraction = *this - P;
    double distance(soustraction.getNorm());
    return distance;
}

void Vector3d::set(double X, double Y, double Z) {
    x = X;
    y = Y;
    z = Z;
}

void Vector3d::setX(double X){
    x = X;
}

void Vector3d::setY(double Y){
    y = Y;
}

void Vector3d::setZ(double Z){
    z = Z;
}

double Vector3d::getX() const{
    return x;
}

double Vector3d::getY() const{
    return y;
}

double Vector3d::getZ() const{
    return z;
}

void Vector3d::normalize() {
    double norm = (*this).getNorm();
    x /= norm;
    y /= norm;
    z /= norm;
}

// OPERATEURS

Vector3d& Vector3d::operator+=(const double d) {
    x += d;
    y += d;
    z += d;
    return *this;
}

Vector3d& Vector3d::operator-=(const double d) {
    return *this += -d;
}

Vector3d& Vector3d::operator*=(const double d) {
    x *= d;
    y *= d;
    z *= d;
    return *this;
}

Vector3d& Vector3d::operator/=(const double d) {
    if (d == 0.0) {
        throw std::domain_error("Division par zéro.");
    }
    Vector3d &P = *this;
    P.x /= d;
    P.y /= d;
    P.z /= d;
    return P;
}

Vector3d& Vector3d::operator+=(const Vector3d &P) {
    x += P.x;
    y += P.y;
    z += P.z;
    return *this;
}

Vector3d& Vector3d::operator-=(const Vector3d &P) {
    Vector3d &V = *this;
    V += -1*P;
    return V;
}

Vector3d& Vector3d::operator=(const Vector3d &P) {
  Vector3d &V = *this;
  V.x = P.x;
  V.y = P.y;
  V.z = P.z;
  return V;
}

// -----------------------------------------------------------

// OPERATEURS

Vector3d operator-(const Vector3d &P) {
  Vector3d V(P);
  V *= -1;
  return V;
}

Vector3d operator+(const Vector3d &P1, const Vector3d &P2) {
    Vector3d P(P1);
    P += P2;
    return P;
}

Vector3d operator-(const Vector3d &P1, const Vector3d &P2){
    Vector3d P(P1);
    P -= P2;
    return P;
}

Vector3d operator*(const double d, const Vector3d &P) {
  Vector3d V(P);
  V *= d;
  return V;
}

Vector3d operator/(const Vector3d &P, const double d) {
  if (d == 0.0) {
    throw std::domain_error("Division par zéro.");
  }
  Vector3d V = (1/d) * P;
  return V;
}

std::ostream& operator<<(std::ostream &str, const Vector3d &P) {
    str.precision(std::numeric_limits<double>::digits10+1);
    str << P.getX() << " " << P.getY() << " " << P.getZ() << std::endl;
    return str;
}
