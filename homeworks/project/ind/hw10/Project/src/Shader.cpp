#include "Shader.h"
using namespace std;
using namespace glm;

	Shader::Shader(){
		program = glCreateProgram();
		unsigned int vert = glCreateShader(GL_VERTEX_SHADER);
		unsigned int frag = glCreateShader(GL_FRAGMENT_SHADER);
		//cout << "default shader\n";
		const char* vtsource =
			"#version 430 core\n"
			"layout(location=0) in vec3 inPos;"
			"void main(){"
			"  gl_Position=vec4(inPos,1.0);"
			"}";
		glShaderSource(vert, 1, &vtsource, 0);
		const char* fgsource =
			"#version 430 core\n"
			"out vec4 fragColor;"
			"void main(){"
			"fragColor= vec4(vec3(1.0,0.0,1.0),1.0);"
			"}";
		glShaderSource(frag, 1, &fgsource, 0);
		glCompileShader(vert);
		glCompileShader(frag);
		glAttachShader(program, vert);
		glAttachShader(program, frag);
		glLinkProgram(program);
		glDeleteShader(vert);
		glDeleteShader(frag);
	}	
	Shader::Shader(const string& vtpath, const string& fgpath) {
		program = glCreateProgram();
		init(vtpath, GL_VERTEX_SHADER);
		init(fgpath, GL_FRAGMENT_SHADER);
		glLinkProgram(program);
	}
	Shader::Shader(const string& vtpath,const string& fgpath,const string& geopath) {
		program = glCreateProgram();
		init(vtpath, GL_VERTEX_SHADER);
		init(geopath, GL_GEOMETRY_SHADER);
		init(fgpath, GL_FRAGMENT_SHADER);
		glLinkProgram(program);
	}
	unsigned int Shader::init(string path, unsigned int type) {
		int success;
		char log[512];
		unsigned int shader = glCreateShader(type);
		string s = loadSource(path);
		const char* source = s.c_str();
		glShaderSource(shader, 1, &source, 0);
		glCompileShader(shader);
		glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
		//cout << type << endl;
		if (!success)
		{
			glGetShaderInfoLog(shader, 512, NULL, log);
			if (type == 35633) {
				cout << "Vertex compile error:" << log <<endl;
			}
			else if (type == 35632) {
				cout << "Fragment compile error:" << log << endl;
			}
			else if (type == 36313) {
				cout << "Geometry compile error:" << log << endl;
			}
		}
		glAttachShader(program, shader);
		glDeleteShader(shader);
		return 0;
	}
	void Shader::set(const char* name, glm::mat4 mat)
	{
		glUniformMatrix4fv(glGetUniformLocation(program, name), 1, false, &mat[0][0]);
	}
	void Shader::set(const char* name, int value) {
		glUniform1i(glGetUniformLocation(program, name), value);
	}
	void Shader::set(const char* name, float value) {
		glUniform1f(glGetUniformLocation(program, name), value);
	}
	void Shader::set(const char* name, glm::vec3 v) {
		glUniform3fv(glGetUniformLocation(program, name), 1, &v[0]);
	}
	void Shader::set(const char* name, glm::vec4 v) {
		glUniform4fv(glGetUniformLocation(program, name), 1, &v[0]);
	}
	void Shader::bind() {
		glUseProgram(program);
	}
	void Shader::unbind() {
		glUseProgram(0);
	}
	string Shader::loadSource(const string& path) {
		ifstream fsm(path);
		string s;
		if (fsm) {
			fileString.clear();
			while (!fsm.eof()) {
				getline(fsm, s);
				fileString += s;
				fileString += "\n";
			}
			fsm.close();
			return fileString;
		}
		else {
			cout << "Shader Source File Not Found" << endl;
			return " ";
		}
	}


