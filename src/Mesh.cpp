#include <cstdlib>
#include <iostream>
#include <math.h>
#include <omp.h>

#include <Mesh.h>
#include <ImplicitFunction.h>

using namespace glm;
using namespace std;

i32vec3 Mesh::get_face(unsigned int i) const {
	glm::i32vec3 face( faces[3*i], faces[3*i+1], faces[3*i+2] );
	return face;
}

vec3 Mesh::get_vertex(unsigned int i) const {
	return vertices[i];
}

vec3 Mesh::get_normal(unsigned int i) const {
	return normals[i];
}


Mesh::Mesh(const char* filename) 
{
	int j = 0;
	unsigned int tmp;
	vector<vec3> normalPerFace;
	float norm;
	vector<float> nv;
	float *n;
	FILE *file;
	int   error;
	float r;

	if((file=fopen(filename,"r"))==NULL) 
	{
		std::cout << "Unable to read : " << filename << std::endl;
	}

	// create mesh
	vertices = vector<vec3>();
    normals  = vector<vec3>();
	faces    = vector<unsigned int>();

	error = fscanf(file,"OFF\n%d %d %d\n",&(nb_vertices),&(nb_faces),&tmp);
	if(error==EOF) 
	{
		std::cout << "Unable to read : " << filename << std::endl;
	}

	vertices.resize(nb_vertices);
    normals.resize(nb_vertices);
	faces.resize(nb_faces*3);

	// reading vertices
	for(int i=0;i<nb_vertices;++i) 
	{
		error = fscanf(file,"%f %f %f\n",&(vertices[i][0]),&(vertices[i][1]),&(vertices[i][2]));
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
		error = fscanf(file,"%d %d %d %d\n",&tmp,&(faces[j]),&(faces[j+1]),&(faces[j+2]));
//		  faces[j]   --;
//        faces[j+1] --;
//        faces[j+2] --;

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

    ComputeNormals();
}

Mesh::~Mesh() {}


vector< vec3 > Mesh::computeBB() const 
{
    vector< vec3 > output;
    
    output.push_back(vertices[0]);
    output.push_back(vertices[0]);
    
	for(int i=1; i<vertices.size(); ++i) 
	{
	    vec3 v = vertices[i];

        output[0] = glm::min(output[0], v);
        output[1] = glm::max(output[1], v);
	}
	
	return output;
}


void Mesh::normalize()
{
    vector<vec3> bb = Mesh::computeBB();
    
	for(int i=0; i<vertices.size(); ++i) 
	{
	    for(int j = 0; j < 3; j++)
	    {
            vertices[i][j] = (vertices[i][j] - bb[0][j]) / (bb[1][j] - bb[0][j]) - 0.5;
	    }
	}
}


void Mesh::CreateCube(Mesh& mesh)
{
    mesh.vertices.push_back(vec3(-1, -1, -1));
    mesh.vertices.push_back(vec3( 1, -1, -1));
    mesh.vertices.push_back(vec3( 1,  1, -1));
    mesh.vertices.push_back(vec3(-1,  1, -1));

    mesh.vertices.push_back(vec3(-1, -1, -1));
    mesh.vertices.push_back(vec3(-1,  1, -1));
    mesh.vertices.push_back(vec3(-1,  1,  1));
    mesh.vertices.push_back(vec3(-1, -1,  1));

    mesh.vertices.push_back(vec3(-1, -1, -1));
    mesh.vertices.push_back(vec3(-1, -1,  1));
    mesh.vertices.push_back(vec3( 1, -1,  1));
    mesh.vertices.push_back(vec3( 1, -1, -1));

    mesh.vertices.push_back(vec3( 1,  1,  1));
    mesh.vertices.push_back(vec3(-1,  1,  1));
    mesh.vertices.push_back(vec3(-1, -1,  1));
    mesh.vertices.push_back(vec3( 1, -1,  1));

    mesh.vertices.push_back(vec3( 1,  1,  1));
    mesh.vertices.push_back(vec3( 1, -1,  1));
    mesh.vertices.push_back(vec3( 1, -1, -1));
    mesh.vertices.push_back(vec3( 1,  1, -1));

    mesh.vertices.push_back(vec3( 1,  1,  1));
    mesh.vertices.push_back(vec3( 1,  1, -1));
    mesh.vertices.push_back(vec3(-1,  1, -1));
    mesh.vertices.push_back(vec3(-1,  1,  1));

    mesh.normals.push_back(vec3(0, 0, -1));
    mesh.normals.push_back(vec3(0, 0, -1));
    mesh.normals.push_back(vec3(0, 0, -1));
    mesh.normals.push_back(vec3(0, 0, -1));

    mesh.normals.push_back(vec3(-1, 0, 0));
    mesh.normals.push_back(vec3(-1, 0, 0));
    mesh.normals.push_back(vec3(-1, 0, 0));
    mesh.normals.push_back(vec3(-1, 0, 0));

    mesh.normals.push_back(vec3(0, -1, 0));
    mesh.normals.push_back(vec3(0, -1, 0));
    mesh.normals.push_back(vec3(0, -1, 0));
    mesh.normals.push_back(vec3(0, -1, 0));

    mesh.normals.push_back(vec3(0, 0,  1));
    mesh.normals.push_back(vec3(0, 0,  1));
    mesh.normals.push_back(vec3(0, 0,  1));
    mesh.normals.push_back(vec3(0, 0,  1));

    mesh.normals.push_back(vec3( 1, 0, 0));
    mesh.normals.push_back(vec3( 1, 0, 0));
    mesh.normals.push_back(vec3( 1, 0, 0));
    mesh.normals.push_back(vec3( 1, 0, 0));

    mesh.normals.push_back(vec3(0,  1, 0));
    mesh.normals.push_back(vec3(0,  1, 0));
    mesh.normals.push_back(vec3(0,  1, 0));
    mesh.normals.push_back(vec3(0,  1, 0));


    mesh.faces.push_back(0);
    mesh.faces.push_back(2);
    mesh.faces.push_back(1);

    mesh.faces.push_back(0);
    mesh.faces.push_back(3);
    mesh.faces.push_back(2);


    mesh.faces.push_back(4);
    mesh.faces.push_back(6);
    mesh.faces.push_back(5);

    mesh.faces.push_back(4);
    mesh.faces.push_back(7);
    mesh.faces.push_back(6);


    mesh.faces.push_back(8);
    mesh.faces.push_back(10);
    mesh.faces.push_back(9);

    mesh.faces.push_back(8);
    mesh.faces.push_back(11);
    mesh.faces.push_back(10);


    mesh.faces.push_back(12);
    mesh.faces.push_back(13);
    mesh.faces.push_back(14);

    mesh.faces.push_back(12);
    mesh.faces.push_back(14);
    mesh.faces.push_back(15);


    mesh.faces.push_back(16);
    mesh.faces.push_back(17);
    mesh.faces.push_back(18);

    mesh.faces.push_back(16);
    mesh.faces.push_back(18);
    mesh.faces.push_back(19);


    mesh.faces.push_back(20);
    mesh.faces.push_back(21);
    mesh.faces.push_back(22);

    mesh.faces.push_back(20);
    mesh.faces.push_back(22);
    mesh.faces.push_back(23);

    mesh.nb_vertices = mesh.vertices.size();
    mesh.nb_faces = mesh.faces.size();
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

            vec3 p(cos(phi)*sin(psi), sin(phi)*sin(psi), cos(psi));
            mesh.vertices.push_back(p);

            mesh.normals.push_back(p);

        }
    }

    for(int i = 0; i < Nu; i++)
    {
        for(int j = 0; j < Nv - 1; j++)
        {
            mesh.faces.push_back( i * Nv + j);
            mesh.faces.push_back((i+1)%Nu * Nv + j);
            mesh.faces.push_back( i * Nv + j+1);

            mesh.faces.push_back( i * Nv + j+1);
            mesh.faces.push_back((i+1)%Nu * Nv + j);
            mesh.faces.push_back((i+1)%Nu * Nv + j + 1);
        }
    }

    mesh.nb_vertices = mesh.vertices.size();
    mesh.nb_faces = mesh.faces.size();
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


                // Cell vertices
                vec3 p000 = vec3(x0, y0, z0);
                vec3 p001 = vec3(x1, y0, z0);
                vec3 p010 = vec3(x0, y1, z0);
                vec3 p011 = vec3(x1, y1, z0);
                vec3 p100 = vec3(x0, y0, z1);
                vec3 p101 = vec3(x1, y0, z1);
                vec3 p110 = vec3(x0, y1, z1);
                vec3 p111 = vec3(x1, y1, z1);

                // Tetrahedra
                vec3 p0[4] =  {p000, p001, p011, p111};
                vec3 p1[4] =  {p000, p011, p010, p111};
                vec3 p2[4] =  {p000, p010, p110, p111};
                vec3 p3[4] =  {p000, p110, p100, p111};
                vec3 p4[4] =  {p000, p100, p101, p111};
                vec3 p5[4] =  {p000, p101, p001, p111};

                // Faces generation
                ProcessTetrahedron(mesh, function, isoValue,  p0);
                ProcessTetrahedron(mesh, function, isoValue,  p1);
                ProcessTetrahedron(mesh, function, isoValue,  p2);
                ProcessTetrahedron(mesh, function, isoValue,  p3);
                ProcessTetrahedron(mesh, function, isoValue,  p4);
                ProcessTetrahedron(mesh, function, isoValue,  p5);
            }
        }
    }

    mesh.nb_vertices = mesh.vertices.size();
    mesh.nb_faces = mesh.faces.size();
}

vec3 findRoot(const ImplicitFunction& function, const float isoValue, const vec3& p0, const vec3& p1, unsigned nb_iter = 10)
{
//    nb_iter = 0;      // Un-comenting this line disable the root finding
    vec3 p00 = p0;
    vec3 p10 = p1;
    if(function.Eval(p0) > function.Eval(p1))
    {
        swap(p00, p10);
    }

    vec3 p = 0.5f*(p00+p10);
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

void Mesh::ProcessTetrahedron(Mesh& mesh, const ImplicitFunction& function, const float isoValue, const vec3 p[])
{
    bool b[4] = {function.Eval(p[0]) > isoValue, function.Eval(p[1]) > isoValue, function.Eval(p[2]) > isoValue, function.Eval(p[3]) > isoValue};


    unsigned int N = mesh.vertices.size();

    // Case where all vertices are above or under the isovalue
    if(!b[0] && !b[1] && !b[2] && !b[3] || b[0] && b[1] && b[2] && b[3])
    {
        return;
    }

    // Case where only one vertex is above or under the isovalue
    for(unsigned int i=0; i<4; i++)
    {
        if(b[i] && !b[(i+1)%4] && !b[(i+2)%4] && !b[(i+3)%4] || !b[i] && b[(i+1)%4] && b[(i+2)%4] && b[(i+3)%4])
        {

            vec3 p0 = findRoot(function, isoValue, p[i], p[(i+1)%4]);
            vec3 p1 = findRoot(function, isoValue, p[i], p[(i+2)%4]);
            vec3 p2 = findRoot(function, isoValue, p[i], p[(i+3)%4]);


            vec3 n0 = glm::normalize(-function.EvalDev(p0));
            vec3 n1 = glm::normalize(-function.EvalDev(p1));
            vec3 n2 = glm::normalize(-function.EvalDev(p2));


            mesh.vertices.push_back(p0);
            mesh.vertices.push_back(p1);
            mesh.vertices.push_back(p2);

            mesh.normals.push_back(n0);
            mesh.normals.push_back(n1);
            mesh.normals.push_back(n2);


            // face orientation according to normal
            if(dot(cross(p1-p0, p2-p0), n0+n1+n2)>=0)
            {
                mesh.faces.push_back(N);
                mesh.faces.push_back(N+1);
                mesh.faces.push_back(N+2);
            }
            else
            {
                mesh.faces.push_back(N);
                mesh.faces.push_back(N+2);
                mesh.faces.push_back(N+1);
            }

            return;
        }
    }

    // Case where two vertices are above or under the isovalue
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
                cerr << "indices are wrong ! " << endl;


            if(b[i] && b[j] && !b[k] && !b[l] || !b[i] && !b[j] && b[k] && b[l])
            {
                vec3 p0 = findRoot(function, isoValue, p[i], p[k]);
                vec3 p1 = findRoot(function, isoValue, p[i], p[l]);
                vec3 p2 = findRoot(function, isoValue, p[j], p[k]);
                vec3 p3 = findRoot(function, isoValue, p[j], p[l]);

                vec3 n0 = glm::normalize(-function.EvalDev(p0));
                vec3 n1 = glm::normalize(-function.EvalDev(p1));
                vec3 n2 = glm::normalize(-function.EvalDev(p2));
                vec3 n3 = glm::normalize(-function.EvalDev(p3));


                mesh.vertices.push_back(p0);
                mesh.vertices.push_back(p1);
                mesh.vertices.push_back(p2);
                mesh.vertices.push_back(p3);


                mesh.normals.push_back(n0);
                mesh.normals.push_back(n1);
                mesh.normals.push_back(n2);
                mesh.normals.push_back(n3);



                // face orientation according to normal
                if(dot(cross(p2-p0, p3-p0), n0+n3+n2)>=0)
                {
                    mesh.faces.push_back(N);
                    mesh.faces.push_back(N+2);
                    mesh.faces.push_back(N+3);

                    mesh.faces.push_back(N);
                    mesh.faces.push_back(N+3);
                    mesh.faces.push_back(N+1);
                }
                else
                {
                    mesh.faces.push_back(N);
                    mesh.faces.push_back(N+3);
                    mesh.faces.push_back(N+2);

                    mesh.faces.push_back(N);
                    mesh.faces.push_back(N+1);
                    mesh.faces.push_back(N+3);
                }


                return;
            }
        }
    }

    cerr << "no solution found in marching tetrahedron !!" << endl;
}


void Mesh::write_obj(const char* filename) const
{
    FILE *file;

    if((file=fopen(filename,"w"))==NULL)
    {
        std::cout << "Unable to open : " << filename << std::endl;
    }

    for(unsigned int i = 0; i < vertices.size(); i++)
    {
        vec3 p = vertices[i];
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
        normals[i] = vec3(0.0);
    }

    for(unsigned int i=0; i<faces.size(); i+=3)
    {
        unsigned int i0 = faces[i+0];
        unsigned int i1 = faces[i+1];
        unsigned int i2 = faces[i+2];

        vec3 p0 = vertices[i0];
        vec3 p1 = vertices[i1];
        vec3 p2 = vertices[i2];

        vec3 d01 = glm::normalize(p1 - p0);
        vec3 d12 = glm::normalize(p2 - p1);
        vec3 d20 = glm::normalize(p0 - p2);

        vec3 faceNormal = glm::normalize(glm::cross(d01, -d20));

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
            cerr << "Error : normal of length 0." << endl;
            continue;
        }

        normals[i] /= l;
    }
}

void Mesh::RemoveDouble(float epsilon)
{
    vector<unsigned int> dbl;
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

    vector<vec3> new_vertices;
    vector<int> corresp;

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


