/*
	Task: Calculate smooth (average) per-vertex normals

	[out] normals - output per-vertex normals
	[in] verts - input per-vertex positions
	[in] faces - triangles (triplets of vertex indices)
	[in] nverts - total number of vertices (# of elements in verts and normals arrays)
	[in] nfaces - total number of faces (# of elements in faces array)
*/

/*
	https://stackoverflow.com/questions/18519586/calculate-normal-per-vertex-opengl
	I assume that vectors in triangle are counter-clockwise ordered
	Also I assume that first three numbers in 'faces' are triangle (0, 1, 2 is triangle, 3, 4, 5 is triangle etc.)
	Instead of each triplet is triangle (0, 1, 2 is triangle, 1, 2, 3 is triange etc.)

	For averagin I used angle between the two edges adjacent to the vertex
*/
#include <vector>
#include <list>
#include <cmath>

class vec3 {
	float x;
	float y;
	float z;

public:
	vec3(float c0, float c1, float c2) {
		x = c0;
		y = c1;
		z = c2;
	}

	vec3 add(vec3& v) {
		return vec3(x + v.x, y + v.y, z + v.z);
	}

	vec3 minus(vec3& v) {
		return vec3(x - v.x, y - v.y, z - v.z);
	}

	vec3 mul(float coef) {
		return vec3(coef*x, coef*y, coef*z);
	}

	vec3 cross(vec3& v) {
		return vec3(y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x);
	}

	float length() {
		return std::sqrt(x*x + y*y + z*z);
	}
};

class triangle {
	int vertices[3];

public:
	triangle(int v0, int v1, int v2) {
		vertices[0] = v0;
		vertices[1] = v1;
		vertices[2] = v2;
	}

	vec3 calculate_normal(const vec3* verts, int index) {
		vec3 a = verts[index];
		vec3 b = verts[vertices[(index+1)%3]];
		vec3 c = verts[vertices[(index+2)%3]];

		// Just temp vectors
		vec3 t1 = b.minus(a);
		vec3 t2 = c.minus(a);

		vec3 result = t1.cross(t2);

		float avg_coef = std::asin(result.length()/(t1.length()*t2.length()));

		result = result.mul(avg_coef);

		return result;
	}
};

/*
	Ok, it is bad solution because use 3x memory but it is simple
	I should use pointers instead but I don't want to make code more complicated
*/
std::vector<std::list<triangle>> preprocessing(const int* faces, int nfaces) {
	std::vector<std::list<triangle>> result (nfaces, std::list<triangle>());

	for (int i = 0; i < nfaces; i += 3) {
		triangle t (faces[i], faces[i+1], faces[i+2]);

		result[faces[i]].push_back(t);
		result[faces[i+1]].push_back(t);
		result[faces[i+2]].push_back(t);
	}

	return result;
}

void calc_mesh_normals(vec3* normals, const vec3* verts, const int* faces, int nverts, int nfaces) {
	std::vector<std::list<triangle>> triangles = preprocessing(faces, nfaces);

	for (int i = 0; i < nverts; ++i)
	{
		normals[i] = vec3 (0.0, 0.0, 0.0);

		for (std::list<triangle>::iterator j = triangles[i].begin(); j != triangles[i].end(); ++j)
		{
			vec3 t = j->calculate_normal(verts, i);
			normals[i].add(t);
		}
	}
}