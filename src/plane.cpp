#include "plane.h"

Plane::Plane() {}

Plane::Plane(const Vector3d &center, const Vector3d &normal):center(center), normal(normal){}

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

Vector3d Plane::getNormal() const {
  return normal;
}

Vector3d Plane::getCenter() const {
  return center;
}

void Plane::display(std::ostream& str) {
  str << "center : " << center << ", normal : " << normal << std::endl;
}
