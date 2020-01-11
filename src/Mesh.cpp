#include <cstdlib>
#include <iostream>
#include <math.h>
#include <omp.h>

#include "Mesh.h"
#include "ImplicitFunction.h"

using namespace glm;
using namespace std;

<<<<<<< HEAD
Mesh::Mesh(const char* filename)
{
	int j = 0;
    unsigned int tmp;
=======

glm::i32vec3 Mesh::get_face(unsigned int i) const {
	glm::i32vec3 face( faces[3*i], faces[3*i+1], faces[3*i+2] );
	return face;
}

glm::vec3 Mesh::get_vertex(unsigned int i) const {
	return vertices[i];
}

glm::vec3 Mesh::get_normal(unsigned int i) const {
	return normals[i];
}


Mesh::Mesh(const char* filename)
{
	int j = 0;
	unsigned int tmp;
    std::vector<glm::vec3> normalPerFace;
	float norm;
    std::vector<float> nv;
>>>>>>> de246948c63828ea82e109e314dfb859070b1a67
	float *n;
	FILE *file;
	int   error;
	float r;

	if((file=fopen(filename,"r"))==NULL)
	{
		std::cout << "Unable to read : " << filename << std::endl;
	}

	// create mesh
<<<<<<< HEAD
    m_positions = vector<vec3>();
    m_normals  = vector<vec3>();
    m_indices    = vector<unsigned int>();

    glm::uint nb_vertices, nb_faces;
=======
	vertices = std::vector<glm::vec3>();
    normals  = std::vector<glm::vec3>();
	faces    = std::vector<unsigned int>();
>>>>>>> de246948c63828ea82e109e314dfb859070b1a67

	error = fscanf(file,"OFF\n%d %d %d\n",&(nb_vertices),&(nb_faces),&tmp);
	if(error==EOF)
	{
		std::cout << "Unable to read : " << filename << std::endl;
	}

    m_positions.resize(nb_vertices);
    m_normals.resize(nb_vertices);
    m_indices.resize(nb_faces*3);

	// reading vertices
	for(int i=0;i<nb_vertices;++i)
	{
        error = fscanf(file,"%f %f %f\n",&(m_positions[i][0]),&(m_positions[i][1]),&(m_positions[i][2]));
		if(error==EOF)
		{
			std::cout << "Unable to read vertices of : " << filename << std::endl;
			exit(EXIT_FAILURE);
		}
	}

	// reading faces
	j = 0;
	for(int i=0;i<nb_faces;++i)
	{
        error = fscanf(file,"%d %d %d %d\n",&tmp,&(m_indices[j]),&(m_indices[j+1]),&(m_indices[j+2]));

		if(error==EOF)
		{
			std::cout << "Unable to read faces of : " << filename << std::endl;
			exit(EXIT_FAILURE);
		}

		if(tmp!=3)
		{
			printf("Error : face %d is not a triangle (%d polygonal face!)\n",i/3,tmp);
			exit(EXIT_FAILURE);
		}
		j += 3;
	}

    fclose(file);

    ColorFill(vec3(0.9));
}



<<<<<<< HEAD
void Mesh::CreateCube(Mesh& mesh)
{
    mesh.m_positions.push_back(vec3(-1, -1, -1));
    mesh.m_positions.push_back(vec3( 1, -1, -1));
    mesh.m_positions.push_back(vec3( 1,  1, -1));
    mesh.m_positions.push_back(vec3(-1,  1, -1));
=======
std::vector< glm::vec3 > Mesh::computeBB() const
{
    std::vector< glm::vec3 > output;
>>>>>>> de246948c63828ea82e109e314dfb859070b1a67

    mesh.m_positions.push_back(vec3(-1, -1, -1));
    mesh.m_positions.push_back(vec3(-1,  1, -1));
    mesh.m_positions.push_back(vec3(-1,  1,  1));
    mesh.m_positions.push_back(vec3(-1, -1,  1));

<<<<<<< HEAD
    mesh.m_positions.push_back(vec3(-1, -1, -1));
    mesh.m_positions.push_back(vec3(-1, -1,  1));
    mesh.m_positions.push_back(vec3( 1, -1,  1));
    mesh.m_positions.push_back(vec3( 1, -1, -1));
=======
	for(int i=1; i<vertices.size(); ++i)
	{
        glm::vec3 v = vertices[i];
>>>>>>> de246948c63828ea82e109e314dfb859070b1a67

    mesh.m_positions.push_back(vec3( 1,  1,  1));
    mesh.m_positions.push_back(vec3(-1,  1,  1));
    mesh.m_positions.push_back(vec3(-1, -1,  1));
    mesh.m_positions.push_back(vec3( 1, -1,  1));

    mesh.m_positions.push_back(vec3( 1,  1,  1));
    mesh.m_positions.push_back(vec3( 1, -1,  1));
    mesh.m_positions.push_back(vec3( 1, -1, -1));
    mesh.m_positions.push_back(vec3( 1,  1, -1));

    mesh.m_positions.push_back(vec3( 1,  1,  1));
    mesh.m_positions.push_back(vec3( 1,  1, -1));
    mesh.m_positions.push_back(vec3(-1,  1, -1));
    mesh.m_positions.push_back(vec3(-1,  1,  1));

<<<<<<< HEAD
    mesh.m_normals.push_back(vec3(0, 0, -1));
    mesh.m_normals.push_back(vec3(0, 0, -1));
    mesh.m_normals.push_back(vec3(0, 0, -1));
    mesh.m_normals.push_back(vec3(0, 0, -1));
=======
void Mesh::normalize()
{
    std::vector<glm::vec3> bb = Mesh::computeBB();
>>>>>>> de246948c63828ea82e109e314dfb859070b1a67

    mesh.m_normals.push_back(vec3(-1, 0, 0));
    mesh.m_normals.push_back(vec3(-1, 0, 0));
    mesh.m_normals.push_back(vec3(-1, 0, 0));
    mesh.m_normals.push_back(vec3(-1, 0, 0));

    mesh.m_normals.push_back(vec3(0, -1, 0));
    mesh.m_normals.push_back(vec3(0, -1, 0));
    mesh.m_normals.push_back(vec3(0, -1, 0));
    mesh.m_normals.push_back(vec3(0, -1, 0));

<<<<<<< HEAD
    mesh.m_normals.push_back(vec3(0, 0,  1));
    mesh.m_normals.push_back(vec3(0, 0,  1));
    mesh.m_normals.push_back(vec3(0, 0,  1));
    mesh.m_normals.push_back(vec3(0, 0,  1));

    mesh.m_normals.push_back(vec3( 1, 0, 0));
    mesh.m_normals.push_back(vec3( 1, 0, 0));
    mesh.m_normals.push_back(vec3( 1, 0, 0));
    mesh.m_normals.push_back(vec3( 1, 0, 0));

    mesh.m_normals.push_back(vec3(0,  1, 0));
    mesh.m_normals.push_back(vec3(0,  1, 0));
    mesh.m_normals.push_back(vec3(0,  1, 0));
    mesh.m_normals.push_back(vec3(0,  1, 0));


    mesh.m_indices.push_back(0);
    mesh.m_indices.push_back(2);
    mesh.m_indices.push_back(1);

    mesh.m_indices.push_back(0);
    mesh.m_indices.push_back(3);
    mesh.m_indices.push_back(2);


    mesh.m_indices.push_back(4);
    mesh.m_indices.push_back(6);
    mesh.m_indices.push_back(5);

    mesh.m_indices.push_back(4);
    mesh.m_indices.push_back(7);
    mesh.m_indices.push_back(6);


    mesh.m_indices.push_back(8);
    mesh.m_indices.push_back(10);
    mesh.m_indices.push_back(9);

    mesh.m_indices.push_back(8);
    mesh.m_indices.push_back(11);
    mesh.m_indices.push_back(10);
=======
void Mesh::CreateCube(Mesh& mesh)
{
    mesh.vertices.push_back(glm::vec3(-1, -1, -1));
    mesh.vertices.push_back(glm::vec3( 1, -1, -1));
    mesh.vertices.push_back(glm::vec3( 1,  1, -1));
    mesh.vertices.push_back(glm::vec3(-1,  1, -1));

    mesh.vertices.push_back(glm::vec3(-1, -1, -1));
    mesh.vertices.push_back(glm::vec3(-1,  1, -1));
    mesh.vertices.push_back(glm::vec3(-1,  1,  1));
    mesh.vertices.push_back(glm::vec3(-1, -1,  1));

    mesh.vertices.push_back(glm::vec3(-1, -1, -1));
    mesh.vertices.push_back(glm::vec3(-1, -1,  1));
    mesh.vertices.push_back(glm::vec3( 1, -1,  1));
    mesh.vertices.push_back(glm::vec3( 1, -1, -1));

    mesh.vertices.push_back(glm::vec3( 1,  1,  1));
    mesh.vertices.push_back(glm::vec3(-1,  1,  1));
    mesh.vertices.push_back(glm::vec3(-1, -1,  1));
    mesh.vertices.push_back(glm::vec3( 1, -1,  1));

    mesh.vertices.push_back(glm::vec3( 1,  1,  1));
    mesh.vertices.push_back(glm::vec3( 1, -1,  1));
    mesh.vertices.push_back(glm::vec3( 1, -1, -1));
    mesh.vertices.push_back(glm::vec3( 1,  1, -1));

    mesh.vertices.push_back(glm::vec3( 1,  1,  1));
    mesh.vertices.push_back(glm::vec3( 1,  1, -1));
    mesh.vertices.push_back(glm::vec3(-1,  1, -1));
    mesh.vertices.push_back(glm::vec3(-1,  1,  1));

    mesh.normals.push_back(glm::vec3(0, 0, -1));
    mesh.normals.push_back(glm::vec3(0, 0, -1));
    mesh.normals.push_back(glm::vec3(0, 0, -1));
    mesh.normals.push_back(glm::vec3(0, 0, -1));

    mesh.normals.push_back(glm::vec3(-1, 0, 0));
    mesh.normals.push_back(glm::vec3(-1, 0, 0));
    mesh.normals.push_back(glm::vec3(-1, 0, 0));
    mesh.normals.push_back(glm::vec3(-1, 0, 0));

    mesh.normals.push_back(glm::vec3(0, -1, 0));
    mesh.normals.push_back(glm::vec3(0, -1, 0));
    mesh.normals.push_back(glm::vec3(0, -1, 0));
    mesh.normals.push_back(glm::vec3(0, -1, 0));

    mesh.normals.push_back(glm::vec3(0, 0,  1));
    mesh.normals.push_back(glm::vec3(0, 0,  1));
    mesh.normals.push_back(glm::vec3(0, 0,  1));
    mesh.normals.push_back(glm::vec3(0, 0,  1));

    mesh.normals.push_back(glm::vec3( 1, 0, 0));
    mesh.normals.push_back(glm::vec3( 1, 0, 0));
    mesh.normals.push_back(glm::vec3( 1, 0, 0));
    mesh.normals.push_back(glm::vec3( 1, 0, 0));

    mesh.normals.push_back(glm::vec3(0,  1, 0));
    mesh.normals.push_back(glm::vec3(0,  1, 0));
    mesh.normals.push_back(glm::vec3(0,  1, 0));
    mesh.normals.push_back(glm::vec3(0,  1, 0));
>>>>>>> de246948c63828ea82e109e314dfb859070b1a67


    mesh.m_indices.push_back(12);
    mesh.m_indices.push_back(13);
    mesh.m_indices.push_back(14);

    mesh.m_indices.push_back(12);
    mesh.m_indices.push_back(14);
    mesh.m_indices.push_back(15);


    mesh.m_indices.push_back(16);
    mesh.m_indices.push_back(17);
    mesh.m_indices.push_back(18);

    mesh.m_indices.push_back(16);
    mesh.m_indices.push_back(18);
    mesh.m_indices.push_back(19);


    mesh.m_indices.push_back(20);
    mesh.m_indices.push_back(21);
    mesh.m_indices.push_back(22);

    mesh.m_indices.push_back(20);
    mesh.m_indices.push_back(22);
    mesh.m_indices.push_back(23);
}

void Mesh::CreateCube2(Mesh& mesh)
{
    for(float x = -1.0; x < 3.0; x += 2.0)
        for(float y = -1.0; y < 3.0; y += 2.0)
            for(float z = -1.0; z < 3.0; z += 2.0)
            {
                mesh.m_positions.push_back(vec3(x, y, z));
                mesh.m_normals.push_back(glm::normalize(vec3(x, y, z)));
            }

    mesh.m_indices.push_back(0);
    mesh.m_indices.push_back(1);
    mesh.m_indices.push_back(3);

    mesh.m_indices.push_back(0);
    mesh.m_indices.push_back(3);
    mesh.m_indices.push_back(2);


    mesh.m_indices.push_back(0);
    mesh.m_indices.push_back(2);
    mesh.m_indices.push_back(6);

    mesh.m_indices.push_back(0);
    mesh.m_indices.push_back(6);
    mesh.m_indices.push_back(4);


    mesh.m_indices.push_back(0);
    mesh.m_indices.push_back(4);
    mesh.m_indices.push_back(5);

    mesh.m_indices.push_back(0);
    mesh.m_indices.push_back(5);
    mesh.m_indices.push_back(1);




    mesh.m_indices.push_back(1);
    mesh.m_indices.push_back(7);
    mesh.m_indices.push_back(3);

    mesh.m_indices.push_back(3);
    mesh.m_indices.push_back(7);
    mesh.m_indices.push_back(2);


    mesh.m_indices.push_back(2);
    mesh.m_indices.push_back(7);
    mesh.m_indices.push_back(6);

    mesh.m_indices.push_back(6);
    mesh.m_indices.push_back(7);
    mesh.m_indices.push_back(4);


    mesh.m_indices.push_back(4);
    mesh.m_indices.push_back(7);
    mesh.m_indices.push_back(5);

    mesh.m_indices.push_back(5);
    mesh.m_indices.push_back(7);
    mesh.m_indices.push_back(1);
}



void Mesh::CreateSphere(Mesh& mesh, unsigned int Nu, unsigned int Nv)
{

    for(int i = 0; i < Nu; i++)
    {
        float u = float(i) / (Nu-1);
        float phi = u * M_PI * 2;

        for(int j = 0; j < Nv; j++)
        {
            float v = float(j) / (Nv-1);
            float psi = v * M_PI;

<<<<<<< HEAD
            vec3 p(cos(phi)*sin(psi), sin(phi)*sin(psi), cos(psi));
            mesh.m_positions.push_back(p);
=======
            glm::vec3 p(cos(phi)*sin(psi), sin(phi)*sin(psi), cos(psi));
            mesh.vertices.push_back(p);
>>>>>>> de246948c63828ea82e109e314dfb859070b1a67

            mesh.m_normals.push_back(p);

        }
    }

    for(int i = 0; i < Nu; i++)
    {
        for(int j = 0; j < Nv - 1; j++)
        {
            mesh.m_indices.push_back( i * Nv + j);
            mesh.m_indices.push_back((i+1)%Nu * Nv + j);
            mesh.m_indices.push_back( i * Nv + j+1);

            mesh.m_indices.push_back( i * Nv + j+1);
            mesh.m_indices.push_back((i+1)%Nu * Nv + j);
            mesh.m_indices.push_back((i+1)%Nu * Nv + j + 1);
        }
    }
}


void Mesh::RemoveDouble(float epsilon)
{
    vector<unsigned int> dbl;
    for(unsigned int i=0; i<m_positions.size(); i++)
        dbl.push_back(i);

    for(unsigned int i=0; i<m_positions.size()-1; i++)
    {
        if(dbl[i] != i)
            continue;

        for(unsigned int j=i+1; j<m_positions.size(); j++)
        {
            if(length(m_positions[i] - m_positions[j]) < epsilon)
            {
                dbl[j] = i;
            }
        }
    }


    for(unsigned int i=0; i<m_positions.size(); i++)
    {
        while(dbl[dbl[i]] != dbl[i])
        {
            dbl[i] = dbl[dbl[i]];
        }
    }

    vector<vec3> new_vertices;
    vector<int> corresp;

    for(unsigned int i=0; i<m_positions.size(); i++)
    {
        if(dbl[i] == i)
        {
            corresp.push_back(new_vertices.size());
            new_vertices.push_back(m_positions[i]);
            continue;
        }

        corresp.push_back(-1);
    }


    for(unsigned int i=0; i<m_positions.size(); i++)
    {
        while(corresp[i] == -1)
        {
            corresp[i] = corresp[dbl[i]];
        }
    }



    for(unsigned int i=0; i<m_indices.size(); i++)
    {
        m_indices[i] = corresp[m_indices[i]];
    }

    m_positions = new_vertices;

}



void Mesh::CreateIsoSurface(  Mesh& mesh
                            , const ImplicitFunction& function
                            , const float isoValue
                            , const float minX
                            , const float maxX
                            , const float minY
                            , const float maxY
                            , const float minZ
                            , const float maxZ
                            , const unsigned int resX
                            , const unsigned int resY
                            , const unsigned int resZ)
{
    for(unsigned int i=0; i < resX; i++)
    {
        float x0 = float(i  )/resX * (maxX - minX) + minX;
        float x1 = float(i+1)/resX * (maxX - minX) + minX;
        for(unsigned int j=0; j < resY; j++)
        {
            float y0 = float(j  )/resY * (maxY - minY) + minY;
            float y1 = float(j+1)/resY * (maxY - minY) + minY;
            for(unsigned int k=0; k < resZ; k++)
            {
                float z0 = float(k  )/resZ * (maxZ - minZ) + minZ;
                float z1 = float(k+1)/resZ * (maxZ - minZ) + minZ;

<<<<<<< HEAD
                vec3 p000 = vec3(x0, y0, z0);
                vec3 p001 = vec3(x1, y0, z0);
                vec3 p010 = vec3(x0, y1, z0);
                vec3 p011 = vec3(x1, y1, z0);
                vec3 p100 = vec3(x0, y0, z1);
                vec3 p101 = vec3(x1, y0, z1);
                vec3 p110 = vec3(x0, y1, z1);
                vec3 p111 = vec3(x1, y1, z1);

                vec3 p0[4] =  {p000, p001, p011, p111};
                vec3 p1[4] =  {p000, p011, p010, p111};
                vec3 p2[4] =  {p000, p010, p110, p111};
                vec3 p3[4] =  {p000, p110, p100, p111};
                vec3 p4[4] =  {p000, p100, p101, p111};
                vec3 p5[4] =  {p000, p101, p001, p111};
=======

                // Cell vertices
                glm::vec3 p000 = vec3(x0, y0, z0);
                glm::vec3 p001 = glm::vec3(x1, y0, z0);
                glm::vec3 p010 = glm::vec3(x0, y1, z0);
                glm::vec3 p011 = glm::vec3(x1, y1, z0);
                glm::vec3 p100 = glm::vec3(x0, y0, z1);
                glm::vec3 p101 = glm::vec3(x1, y0, z1);
                glm::vec3 p110 = glm::vec3(x0, y1, z1);
                glm::vec3 p111 = glm::vec3(x1, y1, z1);

                // Tetrahedra
                glm::vec3 p0[4] =  {p000, p001, p011, p111};
                glm::vec3 p1[4] =  {p000, p011, p010, p111};
                glm::vec3 p2[4] =  {p000, p010, p110, p111};
                glm::vec3 p3[4] =  {p000, p110, p100, p111};
                glm::vec3 p4[4] =  {p000, p100, p101, p111};
                glm::vec3 p5[4] =  {p000, p101, p001, p111};
>>>>>>> de246948c63828ea82e109e314dfb859070b1a67

                ProcessTetrahedron(mesh, function, isoValue,  p0);
                ProcessTetrahedron(mesh, function, isoValue,  p1);
                ProcessTetrahedron(mesh, function, isoValue,  p2);
                ProcessTetrahedron(mesh, function, isoValue,  p3);
                ProcessTetrahedron(mesh, function, isoValue,  p4);
                ProcessTetrahedron(mesh, function, isoValue,  p5);
            }
        }
    }

}

glm::vec3 findRoot(const ImplicitFunction& function, const float isoValue, const glm::vec3& p0, const glm::vec3& p1, unsigned nb_iter = 10)
{
<<<<<<< HEAD
    vec3 p00 = p0;
    vec3 p10 = p1;
=======
//    nb_iter = 0;      // Un-comenting this line disable the root finding
    glm::vec3 p00 = p0;
    glm::vec3 p10 = p1;
>>>>>>> de246948c63828ea82e109e314dfb859070b1a67
    if(function.Eval(p0) > function.Eval(p1))
    {
        swap(p00, p10);
    }

    glm::vec3 p = 0.5f*(p00+p10);
    for(unsigned int iter = 0; iter < nb_iter; iter++)
    {
        if(function.Eval(p) > isoValue)
        {
            p10 = 0.5f * (p00 + p10);
        }
        else
        {
            p00 = 0.5f * (p00 + p10);
        }

        p = 0.5f*(p00+p10);
    }

    return p;
}

void Mesh::ProcessTetrahedron(Mesh& mesh, const ImplicitFunction& function, const float isoValue, const glm::vec3 p[])
{
    bool b[4] = {function.Eval(p[0]) > isoValue, function.Eval(p[1]) > isoValue, function.Eval(p[2]) > isoValue, function.Eval(p[3]) > isoValue};


    unsigned int N = mesh.m_positions.size();
    if(!b[0] && !b[1] && !b[2] && !b[3] || b[0] && b[1] && b[2] && b[3])
    {
        return;
    }

    for(unsigned int i=0; i<4; i++)
    {
        if(b[i] && !b[(i+1)%4] && !b[(i+2)%4] && !b[(i+3)%4] || !b[i] && b[(i+1)%4] && b[(i+2)%4] && b[(i+3)%4])
        {

            glm::vec3 p0 = findRoot(function, isoValue, p[i], p[(i+1)%4]);
            glm::vec3 p1 = findRoot(function, isoValue, p[i], p[(i+2)%4]);
            glm::vec3 p2 = findRoot(function, isoValue, p[i], p[(i+3)%4]);


            glm::vec3 n0 = glm::normalize(-function.EvalDev(p0));
            glm::vec3 n1 = glm::normalize(-function.EvalDev(p1));
            glm::vec3 n2 = glm::normalize(-function.EvalDev(p2));


            mesh.m_positions.push_back(p0);
            mesh.m_positions.push_back(p1);
            mesh.m_positions.push_back(p2);

            mesh.m_normals.push_back(n0);
            mesh.m_normals.push_back(n1);
            mesh.m_normals.push_back(n2);


            if(dot(cross(p1-p0, p2-p0), n0+n1+n2)>0)
            {
                mesh.m_indices.push_back(N);
                mesh.m_indices.push_back(N+1);
                mesh.m_indices.push_back(N+2);
            }
            else
            {
                mesh.m_indices.push_back(N);
                mesh.m_indices.push_back(N+2);
                mesh.m_indices.push_back(N+1);
            }

            return;
        }
    }

    for(unsigned int i=0; i<3; i++)
    {
        for(unsigned int j=i+1; j<4; j++)
        {
            unsigned int k = (i+1)%4;
            if(k == j)
            {
                k += 1;
                k %= 4;
            }

            unsigned int l = (k+1)%4;
            if(l == i)
            {
                l += 1;
                l %= 4;
            }
            if(l == j)
            {
                l += 1;
                l %= 4;
            }

            if(i == k || j == k || i == l || j == l || k == l)
                std::cerr << "indices are wrong ! " << std::endl;


            if(b[i] && b[j] && !b[k] && !b[l] || !b[i] && !b[j] && b[k] && b[l])
            {
                glm::vec3 p0 = findRoot(function, isoValue, p[i], p[k]);
                glm::vec3 p1 = findRoot(function, isoValue, p[i], p[l]);
                glm::vec3 p2 = findRoot(function, isoValue, p[j], p[k]);
                glm::vec3 p3 = findRoot(function, isoValue, p[j], p[l]);

                glm::vec3 n0 = glm::normalize(-function.EvalDev(p0));
                glm::vec3 n1 = glm::normalize(-function.EvalDev(p1));
                glm::vec3 n2 = glm::normalize(-function.EvalDev(p2));
                glm::vec3 n3 = glm::normalize(-function.EvalDev(p3));


                mesh.m_positions.push_back(p0);
                mesh.m_positions.push_back(p1);
                mesh.m_positions.push_back(p2);
                mesh.m_positions.push_back(p3);


                mesh.m_normals.push_back(n0);
                mesh.m_normals.push_back(n1);
                mesh.m_normals.push_back(n2);
                mesh.m_normals.push_back(n3);



                if(dot(cross(p2-p0, p3-p0), n0+n3+n2)>0)
                {
                    mesh.m_indices.push_back(N);
                    mesh.m_indices.push_back(N+2);
                    mesh.m_indices.push_back(N+3);

                    mesh.m_indices.push_back(N);
                    mesh.m_indices.push_back(N+3);
                    mesh.m_indices.push_back(N+1);
                }
                else
                {
                    mesh.m_indices.push_back(N);
                    mesh.m_indices.push_back(N+3);
                    mesh.m_indices.push_back(N+2);

                    mesh.m_indices.push_back(N);
                    mesh.m_indices.push_back(N+1);
                    mesh.m_indices.push_back(N+3);
                }


                return;
            }
        }
    }

    std::cerr << "no solution found in marching tetrahedron !!" << std::endl;
}
<<<<<<< HEAD
=======


void Mesh::write_obj(const char* filename) const
{
    FILE *file;

    if((file=fopen(filename,"w"))==NULL)
    {
        std::cout << "Unable to open : " << filename << std::endl;
    }

    for(unsigned int i = 0; i < vertices.size(); i++)
    {
        glm::vec3 p = vertices[i];
        fprintf(file,"v %f %f %f\n", p[0], p[1], p[2]);
    }

    for(unsigned int i = 0; i < faces.size(); i+=3)
    {
        fprintf(file,"f %i %i %i\n", faces[i]+1, faces[i+1]+1, faces[i+2]+1);
    }
}

std::string Mesh::displayMesh() const{

    for(unsigned int i = 0; i < vertices.size(); i++)
    {
        glm::vec3 p = vertices[i];
        fprintf(file,"v %f %f %f\n", p[0], p[1], p[2]);
    }

    for(unsigned int i = 0; i < faces.size(); i+=3)
    {
        fprintf(file,"f %i %i %i\n", faces[i]+1, faces[i+1]+1, faces[i+2]+1);
    }

}


void Mesh::ComputeNormals()
{
    normals.resize(vertices.size());

    for(unsigned int i=0; i<normals.size(); i++)
    {
        normals[i] = glm::vec3(0.0);
    }

    for(unsigned int i=0; i<faces.size(); i+=3)
    {
        unsigned int i0 = faces[i+0];
        unsigned int i1 = faces[i+1];
        unsigned int i2 = faces[i+2];

        glm::vec3 p0 = vertices[i0];
        glm::vec3 p1 = vertices[i1];
        glm::vec3 p2 = vertices[i2];

        glm::vec3 d01 = glm::normalize(p1 - p0);
        glm::vec3 d12 = glm::normalize(p2 - p1);
        glm::vec3 d20 = glm::normalize(p0 - p2);

        glm::vec3 faceNormal = glm::normalize(glm::cross(d01, -d20));

        float alpha0 = asin(length(glm::cross(d01, -d20)));
        float alpha1 = asin(length(glm::cross(d12, -d01)));
        float alpha2 = asin(length(glm::cross(d20, -d12)));

        normals[i0] += faceNormal * alpha0;
        normals[i1] += faceNormal * alpha1;
        normals[i2] += faceNormal * alpha2;
    }

    for(unsigned int i=0; i<normals.size(); i++)
    {
        float l = length(normals[i]);

        if(l < 1e-5)
        {
            std::cerr << "Error : normal of length 0." << std::endl;
            continue;
        }

        normals[i] /= l;
    }
}

void Mesh::RemoveDouble(float epsilon)
{
    std::vector<unsigned int> dbl;
    for(unsigned int i=0; i<vertices.size(); i++)
        dbl.push_back(i);

    for(unsigned int i=0; i<vertices.size()-1; i++)
    {
        if(dbl[i] != i)
            continue;

        for(unsigned int j=i+1; j<vertices.size(); j++)
        {
            if(length(vertices[i] - vertices[j]) < epsilon)
            {
                dbl[j] = i;
            }
        }
    }


    for(unsigned int i=0; i<vertices.size(); i++)
    {
        while(dbl[dbl[i]] != dbl[i])
        {
            dbl[i] = dbl[dbl[i]];
        }
    }

    std::vector<glm::vec3> new_vertices;
    std::vector<int> corresp;

    for(unsigned int i=0; i<vertices.size(); i++)
    {
        if(dbl[i] == i)
        {
            corresp.push_back(new_vertices.size());
            new_vertices.push_back(vertices[i]);
            continue;
        }

        corresp.push_back(-1);
    }


    for(unsigned int i=0; i<vertices.size(); i++)
    {
        while(corresp[i] == -1)
        {
            corresp[i] = corresp[dbl[i]];
        }
    }



    for(unsigned int i=0; i<faces.size(); i++)
    {
        faces[i] = corresp[faces[i]];
    }

    vertices = new_vertices;

}

>>>>>>> de246948c63828ea82e109e314dfb859070b1a67
