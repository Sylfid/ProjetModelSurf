#ifndef IMPLICIT_FUNCTION_H
#define IMPLICIT_FUNCTION_H

#include <glm/glm.hpp>
#include <glm/gtc/type_precision.hpp> //i32vec3
#include <vector>
#include <string>
#include "cloud.h"


/*
 * Virtual class representing implicit functions to be passed to the marching tetrahedra algorithm (Mesh::CreateIsoSurface()).
 */
class ImplicitFunction
{
public:

    ImplicitFunction();
    ~ImplicitFunction();

    virtual float Eval(glm::vec3 p) const = 0;
    virtual glm::vec3 EvalDev(glm::vec3 p) const = 0;
};


/*
 * Example of class derivation : all you have to do is to insert code inside Eval() and EvalDev() functions in order to compute you function's value.
 */
class MyImplicitFunction : public  ImplicitFunction
{
public:

    MyImplicitFunction(/* data initialization value */);

    MyImplicitFunction(const std::string &filename);
    ~MyImplicitFunction();

    Cloud cloud;
    virtual float Eval(glm::vec3 p) const;
    virtual glm::vec3 EvalDev(glm::vec3 p) const;


private:

// Implicit function data

};



#endif // IMPLICIT_FUNCTION_H
