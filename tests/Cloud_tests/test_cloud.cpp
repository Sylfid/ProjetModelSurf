#include "../../src/plane.h"
#include "../../src/cloud.h"
#include "../../src/vector3d.h"
#include <boost/filesystem.hpp>
#include <assert.h>
#include <math.h>
#include <ctime>

/**
  * \test
  * \brief Programme qui teste la classe cloud
*/
int main () {

    std::string filename = "../../../models/tetrahedron.off";
    if (!(boost::filesystem::exists(filename))) {
        filename = "../models/tetrahedron.off";
    }
    Cloud cloud(filename);
    std::cout << "Test de displayCloud :" << std::endl;
    cloud.displayCloud(std::cout);
    std::cout << "Test de displayPlanes :" << std::endl;
    cloud.displayPlanes(std::cout);

    // Test compute_normal
    std::cout << "------- TEST -------" << std::endl;
    std::cout << std::endl << "Test de compute_normal :" << std::endl;
    Vector3d c(0,0,0);
    Vector3d n1(0,0,1);
    Vector3d n2(0,1,0);
    Vector3d n3(1,0,0);
    Plane p_test;
    p_test.setCenter(c);
    std::vector<Vector3d> nbhd(3);
    nbhd[0] = n1;
    nbhd[1] = n2;
    nbhd[2] = n3;
    compute_normal(nbhd, c, p_test);
    std::cout << "plan : ";
    p_test.display(std::cout);

    assert(p_test.getNormal().getX() == 1);
    assert(p_test.getNormal().getY() == 0);
    assert(p_test.getNormal().getZ() == 0);

    assert(p_test.getU().getX() == 0);
    assert(p_test.getU().getY() == 0);
    assert(p_test.getU().getZ() == 1);

    assert(p_test.getV().getX() == 0);
    assert(p_test.getV().getY() == 1);
    assert(p_test.getV().getZ() == 0);

    std::cout << "Ok" << std::endl;

    // Test compute centroid
    double eps = 0.00000000000000000001;

    std::cout << "------- TEST -------" << std::endl;
    std::cout << "Test de compute_centroid : " << std::endl;
    Plane Pp;
    Vector3d cc = compute_3d_centroid(nbhd);
    std::cout << "centroid : " << cc;
    assert(fabs(cc(0)-0.33333333333333333)<eps);
    assert(fabs(cc(1)-0.33333333333333333)<eps);
    assert(fabs(cc(2)-0.33333333333333333)<eps);
    Pp.setCenter(cc);
    compute_normal(nbhd, cc, Pp);
    std::cout << "Ok" << std::endl;

    // Test construct plans
    std::cout << "------- TEST -------" << std::endl;
    std::cout << "Construction des plans tangents :" << std::endl;
    int K = 4;
    cloud.construct_tangent_planes(K);
    std::cout << "Plans tangents : " <<std::endl;
    cloud.displayPlanes(std::cout);

    std::cout << "FIN !" << std::endl;

    // Test construct plans sur un plus gros fichier
    std::cout << "------- TEST -------" << std::endl;
    int debut = clock();
    std::string f = "../../../models/buddha.off";
    if (!(boost::filesystem::exists(f))) {
        f = "../models/buddha.off";
    }
    Cloud cloudA(f);
    int fin_lecture = clock();
    cloudA.construct_tangent_planes(K);
    int fin_construction = clock();

    std::cout << "TEMPS DEXECUTION :" << std::endl;
    std::cout << "LECTURE OFF FILE : " << (fin_lecture-debut) / double(CLOCKS_PER_SEC)
                << " s" << std::endl;
    std::cout << "SEARCH K-NBHD : " << (fin_construction-fin_lecture) / double(CLOCKS_PER_SEC)
                << " s" << std::endl;
    std::cout << "TOTAL : " << (fin_construction-debut) / double(CLOCKS_PER_SEC)
                << " s" << std::endl;

    return 0;
}
