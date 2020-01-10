#include "ImplicitFunction.h"
#include "../../../src/could.h"

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
    double nearestCenter((self.cloud.getPlanePrecise(0).getCenter()-point).getNorm());
    double actual(0.);
    Vector3d pointAGarder(point - self.cloud.getPlanePrecise(0).getCenter());
    Vector3d normalAGarder(self.cloud.getPlanePrecise(0).getNormal());
    for(int i(0); i<self.cloud.size; i++){
        actual = (self.cloud.getPlanePrecise(i).getCenter()-point).getNorm();
        if(nearestCenter < actual){
            nearestCenter = actual;
            pointAGarder = (point - self.cloud.getPlanePrecise(i).getCenter());
            normalAGarder = self.cloud.getPlanePrecise(i).getNormal();
        }
    }
    return pointAGarder.getScalarProduct(normalAGarder);
}

glm::vec3 MyImplicitFunction::EvalDev(glm::vec3 p) const
{
    // Todo : Implement the gradient of your function here
    Vector3d point(p);
    double nearestCenter((self.cloud.getPlanePrecise(0).getCenter()-point).getNorm());
    double actual(0.);
    Vector3d normalAGarder(self.cloud.getPlanePrecise(0).getNormal());
    for(int i(0); i<self.cloud.size; i++){
        actual = (self.cloud.getPlanePrecise(i).getCenter()-point).getNorm();
        if(nearestCenter < actual){
            nearestCenter = actual;
            normalAGarder = self.cloud.getPlanePrecise(i).getNormal();
        }
    }
    return vec3(normalAGarder.getX(), normalAGarder.getY(); normalAGarder.getZ());
}
