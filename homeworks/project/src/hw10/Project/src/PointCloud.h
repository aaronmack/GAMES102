#pragma once
#include <glad/glad.h>
#include <assimp/Importer.hpp>      
#include <assimp/scene.h>          
#include <assimp/postprocess.h>  
#include <vector>
#include <iostream>
#include <glm/glm.hpp>
using namespace std;
using namespace glm;
class PointCloud
{
public:
	GLuint vao, vbo;
	vector<vec3> posList;
	PointCloud();
	bool load(const string& path);
	void draw();
	void draw(int off, int cnt);
};

