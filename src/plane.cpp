#include "plane.h"

Plane::Plane() {}

Plane::Plane(const Vector3d &c, const Vector3d &n) {
    center = c;
    normal = n;
}

Plane::Plane(const Vector3d &c, const Vector3d &n, const Vector3d &U, const Vector3d &V) {
    center = c;
    normal = n;
    u = U;
    v = V;
}

Plane::~Plane() {}

void Plane::setNormal(const Vector3d &n) {
  normal.setX(n.getX());
  normal.setY(n.getY());
  normal.setZ(n.getZ());
}

void Plane::setCenter(const Vector3d &c) {
  center.setX(c.getX());
  center.setY(c.getY());
  center.setZ(c.getZ());
}

void Plane::setU(const Vector3d &U) {
    u.setX(U.getX());
    u.setY(U.getY());
    u.setZ(U.getZ());
}

void Plane::setV(const Vector3d &V) {
    v.setX(V.getX());
    v.setY(V.getY());
    v.setZ(V.getZ());
}

Vector3d Plane::getNormal() const {
  return normal;
}

Vector3d Plane::getCenter() const {
  return center;
}

Vector3d Plane::getU() const {
  return u;
}

Vector3d Plane::getV() const {
  return v;
}

void Plane::display(std::ostream& str) {
    str.precision(std::numeric_limits<double>::digits10+1);
  str << "center : " << center << "normal : " << normal << "u : " << u
        << "v : " << v << std::endl;
}
