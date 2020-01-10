#ifndef MESH_H
#define MESH_H

#include <glm/glm.hpp>
#include <glm/gtc/type_precision.hpp> //i32vec3
#include <vector>
#include <string>

class ImplicitFunction;

class Mesh
{
 public:

  Mesh(){}
  Mesh(const char* filename);   // filename has to be the name of an OFF file
  ~Mesh();

  // length 
  unsigned int  nb_vertices;
  unsigned int  nb_faces;
  
  // data
  std::vector< glm::vec3 > vertices;                // Vertices position
  std::vector< glm::vec3 > normals;                 // Vertices normal
  std::vector< unsigned int > faces;                // Triplets of indices representing faces

  // accessors
  glm::i32vec3 get_face(unsigned int i) const;
  glm::vec3 get_vertex(unsigned int i) const;
  glm::vec3 get_normal(unsigned int i) const;

  // utils
  std::vector< glm::vec3 > computeBB() const ;      // Computes the minimal bounding box containing the mesh
  void normalize();                                 // Centers and scales the mesh so that is fits inside the unit cube
  void ComputeNormals();                            // Computes a new normal for each vertex based on its incident faces
  void RemoveDouble(float epsilon = 1e-5);          // Removes each vertex which is closer than epsilon to an other vertex

  // primitives
  static void CreateCube(Mesh& mesh);
  static void CreateSphere(Mesh& mesh, unsigned int Nu = 100, unsigned int Nv = 50);

  // i/o
  void write_obj(const char* filename) const;

  // marching tetrahedra
  static void CreateIsoSurface(Mesh& mesh, const ImplicitFunction& function, const float isoValue = 0.5
          , const float minX = -1.0, const float maxX = 1.0
          , const float minY = -1.0, const float maxY = 1.0
          , const float minZ = -1.0, const float maxZ = 1.0
          , const unsigned int resX = 100
          , const unsigned int resY = 100
          , const unsigned int resZ = 100);

  static void CreateIsoSurface(Mesh& mesh, const ImplicitFunction& function, const float isoValue
          , const glm::vec3 BBmin
          , const glm::vec3 BBmax
          , const glm::ivec3 res)
  {CreateIsoSurface(mesh, function, isoValue, BBmin.x, BBmax.x, BBmin.y, BBmax.y, BBmin.z, BBmax.z, res.x, res.y, res.z);}

  static void ProcessTetrahedron(Mesh& mesh, const ImplicitFunction& function, const float isoValue, const glm::vec3 p[]);
};

#endif // MESH_H
