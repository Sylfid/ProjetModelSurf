#include "../src/plane.h"
#include "../src/vector3d.h"
#include <assert.h>

/**
  * \test
  * \brief Programme qui teste la classe Plane
*/
int main () {

    std::cout << "Constructeur avec initialisation : " << std::endl;
    Vector3d c;
    Vector3d n(0,0,1);
    // Vector3d u(1,0,0);
    // Vector3d v(0,1,0);
    // std::cout << "centre : " << c;
    // std::cout << "normal : " << n;
    // std::cout << "u : " << u;
    // std::cout << "v : " << v;

    Plane P(c,n);
    // std::cout << "Composantes du Plan P :" << std::endl;
    // P.display(std::cout);

    Vector3d c0 = P.getCenter();
    assert(c0.getX() == c.getX());
    assert(c0.getY() == c.getY());
    assert(c0.getZ() == c.getZ());

    Vector3d n0 = P.getNormal();
    assert(n0.getX() == n.getX());
    assert(n0.getY() == n.getY());
    assert(n0.getZ() == n.getZ());

    // Vector3d uco0 = P.getU();
    // assert(uco0.getX() == u.getX());
    // assert(uco0.getY() == u.getY());
    // assert(uco0.getZ() == u.getZ());
    //
    // Vector3d vco0 = P.getV();
    // assert(vco0.getX() == v.getX());
    // assert(vco0.getY() == v.getY());
    // assert(vco0.getZ() == v.getZ());

    std::cout << "initialisation ok" << std::endl;

    c.setX(6);
    assert(P.getCenter().getX() == 0);

    assert(c0.getX() == 0);
    assert(c0.getY() == c.getY());
    assert(c0.getZ() == c.getZ());

    assert(n0.getX() == n.getX());
    assert(n0.getY() == n.getY());
    assert(n0.getZ() == n.getZ());

    // assert(uco0.getX() == u.getX());
    // assert(uco0.getY() == u.getY());
    // assert(uco0.getZ() == u.getZ());
    //
    // assert(vco0.getX() == v.getX());
    // assert(vco0.getY() == v.getY());
    // assert(vco0.getZ() == v.getZ());
    printf("La modification des vecteurs ne modifie pas P\n");

    std::cout << "\nConstructeur par défaut\n";
    Plane P1;
    Vector3d n1(0,0,1);
    // Vector3d u1(1,1,0);
    // Vector3d v1(0,1,-1);
    // std::cout << "centre : " << c;
    // std::cout << "normal : " << n1;
    // std::cout << "u : " << u1;
    // std::cout << "v : " << v1;
    P1.setCenter(c);
    P1.setNormal(n1);
    // P1.setU(u1);
    // P1.setV(v1);
    // std::cout << "Plan : \n";
    // P1.display(std::cout);

    Vector3d cc1 = P1.getCenter();
    assert(cc1.getX() == c.getX());
    assert(cc1.getY() == c.getY());
    assert(cc1.getZ() == c.getZ());

    Vector3d nc1 = P1.getNormal();
    assert(nc1.getX() == n1.getX());
    assert(nc1.getY() == n1.getY());
    assert(nc1.getZ() == n1.getZ());

    // Vector3d uco1 = P1.getU();
    // assert(uco1.getX() == u1.getX());
    // assert(uco1.getY() == u1.getY());
    // assert(uco1.getZ() == u1.getZ());
    //
    // Vector3d vco1 = P1.getV();
    // assert(vco1.getX() == v1.getX());
    // assert(vco1.getY() == v1.getY());
    // assert(vco1.getZ() == v1.getZ());
    std::cout << "OK" << std::endl;

    std::cout << "Test sur les setters ok" << std::endl;
    std::cout << "Test sur les getters ok" << std::endl;

    Vector3d cen = P1.getCenter();

    cen.set(-1, 3, 9.2);
    assert(P1.getCenter().getX() == 6);
    assert(P1.getCenter().getY() == 0);
    assert(P1.getCenter().getZ() == 0);
    std::cout << "Test sur getCenter : ok" << std::endl;

    return 0;
}
