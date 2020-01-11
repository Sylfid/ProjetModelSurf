#ifndef MESH_H
#define MESH_H

#include <glm/glm.hpp>
#include <glm/gtc/type_precision.hpp> //i32vec3
#include <vector>
#include <string>

#include "AbstractMesh.h"

class ImplicitFunction;

/**
  * \brief Construit un maillage et implante l'algorithme d'une version du
            marching cube : le marching tetrahedra pour une fonction implicite
            donnée (partie 3.5 Contour Tracing de l'article Hoppe92).
*/
class Mesh : public AbstractMesh {
public:
    /**
      * \brief Constructeur par défaut de la classe Mesh
    */
    Mesh(){} // Empty constructor

    /**
      * \brief Supprimer les points multiples
    */
    void RemoveDouble(float epsilon = 1e-5); /// Remove duplicated points

    /**
      * \brief Implante l'algorithme du marching tetrahedra dans une région délimitée
      * \param function la fonction implicite dont il faut extraire l'isovaleur
      * \param isoValue l'isovaleur
      * \param minX la coordonnée limite min en x (délimitant la région)
      * \param maxX la coordonnée limite mac en x (délimitant la région)
      * \param minY la coordonnée limite min en y (délimitant la région)
      * \param maxY la coordonnée limite max en y (délimitant la région)
      * \param minZ la coordonnée limite min en z (délimitant la région)
      * \param maxZ la coordonnée limite max en z (délimitant la région)
      * \param resX resolution en x
      * \param resY resolution en y
      * \param resZ resolution en z
    */
    void CreateIsoSurface(const ImplicitFunction& function, const float isoValue
            , const float minX, const float maxX
            , const float minY, const float maxY
            , const float minZ, const float maxZ
            , const unsigned int resX = 100
            , const unsigned int resY = 100
            , const unsigned int resZ = 100); /// Implements the marching cube algorithm

    /**
      * \brief Construction de l’isosurface sur un tétraèdre
      * \param function la fonction implicite dont il faut extraire l'isovaleur
      * \param isoValue l'isovaleur
      * \param p le tétraèdre
    */
    void ProcessTetrahedron(const ImplicitFunction& function, const float isoValue, const glm::vec3 p[]);
    /// Processes a tetrahedron during marching tetrahedron algorithm


    /* =========================================================================
                            ANCIENNE VERSION (ULYSSE VIMONT)
    ========================================================================= */

    // Mesh(const char* filename);     /// Imports a mesh from an OFF File
    //
    // // utils
    // void RemoveDouble(float epsilon = 1e-5);        /// Remove duplicated points
    //
    //
    // // primitives generation
    // static void CreateCube(Mesh& mesh);             /// Creates a cube (vertices duplicated)
    // static void CreateCube2(Mesh& mesh);            /// Creates a cube (vertices not duplicated)
    // static void CreateSphere(Mesh& mesh, unsigned int Nu = 100, unsigned int Nv = 50);  /// Creates a UV sphere

    // // marching tetrahedra
    // static void CreateIsoSurface(Mesh& mesh, const ImplicitFunction& function, const float isoValue = 0.5
    //         , const float minX = -1.0, const float maxX = 1.0
    //         , const float minY = -1.0, const float maxY = 1.0
    //         , const float minZ = -1.0, const float maxZ = 1.0
    //         , const unsigned int resX = 100
    //         , const unsigned int resY = 100
    //         , const unsigned int resZ = 100);           /// Implements the marching cube algorithm
    //
    // static void ProcessTetrahedron(Mesh& mesh, const ImplicitFunction& function, const float isoValue, const glm::vec3 p[]);        /// Processes a tetrahedron during marching tetrahedron algorithm
};

#endif // MESH_H
