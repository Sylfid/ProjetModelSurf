#ifndef IMPLICIT_FUNCTION_H
#define IMPLICIT_FUNCTION_H

#include <glm/glm.hpp>
#include <glm/gtc/type_precision.hpp> //i32vec3
#include <vector>
#include <string>
#include "cloud.h"
#include "consistent_orientation.h"
#include <math.h>
#ifdef INFINITY
/* INFINITY is supported */
#endif

class ImplicitFunction
{
public:

    ImplicitFunction();
    ~ImplicitFunction();

    virtual float Eval(glm::vec3 p) const;
    virtual glm::vec3 EvalDev(glm::vec3 p) const;

    virtual glm::vec3 EvalDevFiniteDiff(glm::vec3 p) const;
};

// ======================== DEBUT SignedDistanceFunction========================

/**
  * \brief La function distance signée décrit dans l'article de Hoppe 92 est une
            fonction implicite.
*/
class SignedDistanceFunction : public ImplicitFunction
{
private:
    Cloud pointcloud; /*!< la nuage de point à reconstruire */

public:
    /**
      * \brief Constructeur pour la fonction distance signée
      * \param filename le nom du fichier .OFF qui contient le nuage de points à
                récupérer
      * \param K entier pour la recherche des K plus proches voisins, utilisé
                pour l'estimation des plans tangents
      * \param delta paramètre de bruit (par exemple : lié à la précision d'un
                laser)
    */
    SignedDistanceFunction(const std::string &filename, const int K, const double delta);
    /**
      * \brief Destructeur
    */
    ~SignedDistanceFunction();

    /**
      * \brief Evalue la fonction distance signée au point p.
      * \param p le point en lequel la fonction distance est évaluée.
    */
    virtual float Eval(glm::vec3 p) const;
    virtual glm::vec3 EvalDev(glm::vec3 p) const;

    int getSize() const;

    double getMinX() const {return pointcloud.getMinX();};
    double getMinY() const {return pointcloud.getMinY();};
    double getMinZ() const {return pointcloud.getMinZ();};
    double getMaxX() const {return pointcloud.getMaxX();};
    double getMaxY() const {return pointcloud.getMaxY();};
    double getMaxZ() const {return pointcloud.getMaxZ();};

    double getRhoPlusDelta() const {return pointcloud.getRhoPlusDelta();};

    Cloud getPointCloud() const;
    Cloud &getPointCloud();

};

// ======================== FIN SignedDistanceFunction==========================


class SphereFunction : public  ImplicitFunction
{
public:

    SphereFunction(const glm::vec3& centre, const float radius);
    ~SphereFunction();

    virtual float Eval(glm::vec3 p) const;
    virtual glm::vec3 EvalDev(glm::vec3 p) const;


private:

    glm::vec3 m_centre;
    float m_radius;
};


class AddFunction : public  ImplicitFunction
{
public:

    AddFunction(const ImplicitFunction& op0, const ImplicitFunction& op1);
    ~AddFunction();

    virtual float Eval(glm::vec3 p) const;
    virtual glm::vec3 EvalDev(glm::vec3 p) const;


private:

    const ImplicitFunction& m_op0;
    const ImplicitFunction& m_op1;
};

class MulFunction : public  ImplicitFunction
{
public:

    MulFunction(const ImplicitFunction& op0, const ImplicitFunction& op1);
    ~MulFunction();

    virtual float Eval(glm::vec3 p) const;
    virtual glm::vec3 EvalDev(glm::vec3 p) const;


private:

    const ImplicitFunction& m_op0;
    const ImplicitFunction& m_op1;
};

class MinFunction : public  ImplicitFunction
{
public:

    MinFunction(const ImplicitFunction& op0, const ImplicitFunction& op1);
    ~MinFunction();

    virtual float Eval(glm::vec3 p) const;
    virtual glm::vec3 EvalDev(glm::vec3 p) const;


private:

    const ImplicitFunction& m_op0;
    const ImplicitFunction& m_op1;
};


class MaxFunction : public  ImplicitFunction
{
public:

    MaxFunction(const ImplicitFunction& op0, const ImplicitFunction& op1);
    ~MaxFunction();

    virtual float Eval(glm::vec3 p) const;
    virtual glm::vec3 EvalDev(glm::vec3 p) const;


private:

    const ImplicitFunction& m_op0;
    const ImplicitFunction& m_op1;
};

class BlendFunction : public  ImplicitFunction
{
public:

    BlendFunction(const ImplicitFunction& op0, const ImplicitFunction& op1, const ImplicitFunction& weight);
    ~BlendFunction();

    virtual float Eval(glm::vec3 p) const;
    virtual glm::vec3 EvalDev(glm::vec3 p) const;


private:

    const ImplicitFunction& m_op0;
    const ImplicitFunction& m_op1;
    const ImplicitFunction& m_weight;
};


class ScalProdFunction : public  ImplicitFunction
{
public:

    ScalProdFunction(const float scalar, const ImplicitFunction& func);
    ~ScalProdFunction();

    virtual float Eval(glm::vec3 p) const;
    virtual glm::vec3 EvalDev(glm::vec3 p) const;


private:

    const float m_scalar;
    const ImplicitFunction& m_func;
};




class SegmentFunction : public  ImplicitFunction
{
public:

    SegmentFunction(const glm::vec3& p0, const float r0, const glm::vec3& p1, const float r1);
    ~SegmentFunction();

    virtual float Eval(glm::vec3 p) const;
    virtual glm::vec3 EvalDev(glm::vec3 p) const;


private:

    glm::vec3 m_p0;
    float m_r0;
    glm::vec3 m_p1;
    float m_r1;
};




class CircleFunction : public  ImplicitFunction
{
public:

    CircleFunction(const glm::vec3& centre, const float radius_big, const float radius_small, const glm::vec3& normal);
    ~CircleFunction();

    virtual float Eval(glm::vec3 p) const;
    virtual glm::vec3 EvalDev(glm::vec3 p) const;


private:

    glm::vec3 m_centre;
    float m_radius_big;
    float m_radius_small;
    glm::vec3 m_normal;
};



class PerlinFunction : public  ImplicitFunction
{
public:

    PerlinFunction(const glm::uint nb_octave = 3, const float persistency = 0.5, const float scale = 1.0, const glm::vec3& displacement = glm::vec3(0.0));
    ~PerlinFunction();

    virtual float Eval(glm::vec3 p) const;
    virtual glm::vec3 EvalDev(glm::vec3 p) const;


private:

    glm::uint m_nb_octave;
    float m_persistency;

    float m_scale;
    glm::vec3 m_displacement;
};




class BarthFunction : public  ImplicitFunction
{
public:

    BarthFunction();
    ~BarthFunction();

    virtual float Eval(glm::vec3 p) const;
    virtual glm::vec3 EvalDev(glm::vec3 p) const;


private:
};







#endif // IMPLICIT_FUNCTION_H
