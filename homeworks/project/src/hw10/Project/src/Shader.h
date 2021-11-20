#pragma once
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <iostream>
#include <fstream>
#include <string>

using namespace std;
using namespace glm;

class Shader {
public:
	unsigned int program;
	const char* vtsource;
	const char* fgsource;
	string  vts, fgs;
	string fileString;
	Shader();
	Shader(const string& vtpath, const string& fgpath);
	Shader(const string& vtpath, const string& fgpath, const string& gepath);
	//Shader(string vtpath, string fgpath );
	void set(const char* name, glm::mat4 mat);
	void set(const char* name, int value);
	void set(const char* name, float value);
	void set(const char* name, glm::vec3 v);
	void set(const char* name, glm::vec4 v);
	void bind();
	void unbind();
	string loadSource(const string& path);
	unsigned int init(string vtpath, unsigned int type);
};

