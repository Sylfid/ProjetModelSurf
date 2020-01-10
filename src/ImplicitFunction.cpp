#include "ImplicitFunction.h"
#include "could.h"

#include <iostream>
#include <math.h>

using namespace glm;
using namespace std;



ImplicitFunction::ImplicitFunction()
{

}

ImplicitFunction::~ImplicitFunction()
{

}

MyImplicitFunction::MyImplicitFunction(const std::string &filename)
{
    Cloud newCloud(&filename);
    newCloudconstruct_tangent_planes(5);
    cloud = newCloud;
}

MyImplicitFunction::~MyImplicitFunction()
{

}

float MyImplicitFunction::Eval(glm::vec3 p) const
{
    // Todo : Implement your function here
    Vector3d point(p);
    double nearestCenter((cloud.getPlanePrecise(0).getCenter()-point).getNorm());
    double actual(0.);
    Vector3d pointAGarder(point - cloud.getPlanePrecise(0).getCenter());
    Vector3d normalAGarder(cloud.getPlanePrecise(0).getNormal());
    for(int i(0); i<cloud.size; i++){
        actual = (cloud.getPlanePrecise(i).getCenter()-point).getNorm();
        if(nearestCenter < actual){
            nearestCenter = actual;
            pointAGarder = (point - cloud.getPlanePrecise(i).getCenter());
            normalAGarder = cloud.getPlanePrecise(i).getNormal();
        }
    }
    return pointAGarder.getScalarProduct(normalAGarder);
}

glm::vec3 MyImplicitFunction::EvalDev(glm::vec3 p) const
{
    // Todo : Implement the gradient of your function here
    Vector3d point(p);
    double nearestCenter((cloud.getPlanePrecise(0).getCenter()-point).getNorm());
    double actual(0.);
    Vector3d normalAGarder(cloud.getPlanePrecise(0).getNormal());
    for(int i(0); i<cloud.size; i++){
        actual = (cloud.getPlanePrecise(i).getCenter()-point).getNorm();
        if(nearestCenter < actual){
            nearestCenter = actual;
            normalAGarder = cloud.getPlanePrecise(i).getNormal();
        }
    }
    return vec3(normalAGarder.getX(), normalAGarder.getY(); normalAGarder.getZ());
}
