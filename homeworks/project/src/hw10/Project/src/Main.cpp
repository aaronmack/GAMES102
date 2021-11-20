#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/string_cast.hpp>
#include "Shader.h"
#include <vector>
#include <iostream>
#include <map>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <list>
#include <vector>
#include "PointCloud.h"
#include <fade3d/Fade_3D.h>
#include <Voro/voro++.hh>
#define CONVHULL_3D_ENABLE
#include <ConvexHull/convhull_3d.h>
using namespace std;
using namespace glm;
using namespace FADE3D;
using namespace voro;

vector<float> mousex(2);
vector<float> mousey(2);
float  WIDTH = 500;
float  HEIGHT = 500;
bool mouse_press = false;
extern float frame = 0;
float rotY =2.7, rotX = -0.2;
float zoom = 6;
#define PI 3.1415926
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);
void mouse_callback(GLFWwindow* window, int button, int action, int mods);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void mouseInput(GLFWwindow* window);

void generateBuffer(GLuint& vao, GLuint& vbo, vector<double>& list) {
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);
    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, list.size() * 8, list.data(), GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 3, GL_DOUBLE, false, 0, (void*)0);
    glEnableVertexAttribArray(0);
}

struct mTriangle {
    int vex[3];
};

int main()
{
    glfwInit();
    GLFWwindow* window = glfwCreateWindow(WIDTH, HEIGHT, " ", 0, 0);
    glfwMakeContextCurrent(window);
    glfwSetMouseButtonCallback(window, mouse_callback);
    glfwSetScrollCallback(window, scroll_callback);
    gladLoadGLLoader((GLADloadproc)glfwGetProcAddress);
    glfwSwapInterval(1);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_POINT_SIZE);
    glPointSize(2);
    glEnable(GL_LINE_WIDTH);
    glLineWidth(1);
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(1, 1);
    PointCloud pcloud;
    pcloud.load("../model/Arma.obj");
    Shader shade("../glsl/color.vert", "../glsl/color.frag");

    ///=======================convex hull===================
    int vcnt = pcloud.posList.size();
    vector<bool> onConvex(vcnt,false);  
    vector<bool> onSurf(vcnt, false);

    ch_vertex* vertices;   
    vertices = (ch_vertex*)malloc(vcnt* sizeof(ch_vertex));
   
    for (int i = 0; i < vcnt; i++) {   
        vertices[i].x = pcloud.posList[i].x;
        vertices[i].y = pcloud.posList[i].y;
        vertices[i].z = pcloud.posList[i].z;
    }
    int* faceVert = NULL;
    int fcnt;
    ///============== CONVEX HULL =======================================
    convhull_3d_build(vertices,vcnt, &faceVert, &fcnt);
    ///================================================================
    vector<mTriangle> mtrig(fcnt);
    for (int i = 0; i < fcnt; i++) {
        for (int j = 0; j < 3; j++) {
            int id = i * 3 + j;  // this id is global id of convex vert
            // each face has 3 vert, 
            onConvex[faceVert[id]] = true;
            mtrig[i].vex[j]= faceVert[id];
        }
    
    }
    ///to get face[i]'s vert, use  faceVert[i*3] ,[i*3+1] ,[i*3+2] 
    /// ===find neighbor============================= 
    vector<vector<int>> neighborFace(vcnt);  //neighbFace of v[i]
    for (int k = 0; k < vcnt; k++) {
        for (int i = 0; i < fcnt; i++) {
            for (int j = 0; j < 3; j++) {
                int id = i * 3 + j;
                if (faceVert[id] == k) {
                    neighborFace[k].push_back(i);
                    break;
                }
            }     
        }
    } 
    ///============Voronoi=================================================
    vector<int> neigh, f_vert;
    const double s_m = 4;
    // Set up the number of blocks that the container is divided into
    const int n_s = 4;
    double x, y, z;    
    container con(-s_m, s_m, -s_m, s_m, -s_m, s_m, n_s, n_s, n_s,
        false, false, false, 8);

    for (int i = 0; i < pcloud.posList.size(); i++) {
        vec3 v = pcloud.posList[i];
        x=v.x, y=v.y, z=v.z;
        con.put(i, x, y, z);
    }
    c_loop_all cloop(con);
    vector<vector<double>> voroGroup(pcloud.posList.size());
    voronoicell_neighbor cell;
    if (cloop.start()) do if (con.compute_cell(cell, cloop)) {       
        vector<double> tmp;
        cloop.pos(x, y, z); 
        int id = cloop.pid();  //id for each vcell
        cell.neighbors(neigh);
        cell.face_vertices(f_vert);
        cell.vertices(x, y, z, tmp);  //verts for one vcell
        //nth  voro verts group
        voroGroup[id].insert(voroGroup[id].end(), tmp.begin(), tmp.end());
    } while (cloop.inc());
    

    ///============Delaunay==========================================
    Fade_3D DT;
    for (int i = 0; i < pcloud.posList.size(); i++) {
        vec3 v = pcloud.posList[i];
        Point3 p(v.x, v.y, v.z);
        p.onSurf = true;     
        DT.insert(p);
    
        vec3 n_plus;
        vector<double> sp(voroGroup[i].size());
        ///======== pole+ ==================================
        for (int j = 0; j < voroGroup[i].size(); j += 3) {
            sp[j] = length(vec3(voroGroup[i][j], voroGroup[i][j + 1], voroGroup[i][j + 2])
                - vec3(p.x(), p.y(), p.z()));
        }
        auto it = std::max_element(sp.begin(), sp.end());
        int id = std::distance(sp.begin(), it);  ///the id of max value

        Point3  pole0(voroGroup[i][id], voroGroup[i][id + 1], voroGroup[i][id + 2]);
        pole0.onSurf = false;
        DT.insert(pole0);
        ///===============in Convex==================================      
        if (!onConvex[i]) {              
            n_plus = normalize(vec3(p.x(), p.y(), p.z()) -
                     vec3(voroGroup[i][id], voroGroup[i][id + 1], voroGroup[i][id + 2]));       
        }else {
            int nbCnt = neighborFace[i].size();
            n_plus = vec3(0);
            for (int j = 0; j < nbCnt; j++) {
                int id = neighborFace[i][j];
                int v0=  mtrig[id].vex[0];
                int v1 = mtrig[id].vex[1];
                int v2 = mtrig[id].vex[2];
                vec3 a= pcloud.posList[v1]-  pcloud.posList[v0];
                vec3 b = pcloud.posList[v2] - pcloud.posList[v0];
                n_plus +=normalize(cross(a,b));
            }
            n_plus/=float(nbCnt);
            
        }
        /// ============ pole - =============================
        for (int j = 0; j < voroGroup[i].size(); j += 3) {
            sp[j] = glm::dot(vec3(voroGroup[i][j], voroGroup[i][j + 1], voroGroup[i][j + 2]),
                -n_plus);
        }
        it = std::max_element(sp.begin(), sp.end());
        id = std::distance(sp.begin(), it);   ///the id of max value
        Point3  pole1(voroGroup[i][id], voroGroup[i][id + 1], voroGroup[i][id + 2]);
        pole1.onSurf = false;
        DT.insert(pole1);

    }
    vector<Tet3*> tetra;
    DT.getTetrahedra(tetra);
    vector<double> plist;
    ///===push related vertices===============
    for (int s = 0; s < tetra.size(); s++) {
        for (int i = 0; i< 4; i++) {
            int a, b, c;
            tetra[s]->getFacetIndices(i, a, b, c);
            Point3* p0 = tetra[s]->getCorner(a);           
            Point3* p1 = tetra[s]->getCorner(b);
            Point3* p2 = tetra[s]->getCorner(c);
            if ((p0->onSurf && p1->onSurf && p2->onSurf)) {
                plist.push_back(p0->x());
                plist.push_back(p0->y());
                plist.push_back(p0->z());
                plist.push_back(p1->x());
                plist.push_back(p1->y());
                plist.push_back(p1->z());
                plist.push_back(p2->x());
                plist.push_back(p2->y());
                plist.push_back(p2->z());
            }          
        }
    }
    GLuint vao, vbo;
    generateBuffer(vao, vbo, plist);

  
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        mouseInput(window);
        glClearColor(0.5, 0.5, 0.55, 1.0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        shade.bind();
        mat4 model = mat4(1.0), view, proj;
        model = rotate(model, -rotX, vec3(1, 0, 0));
        model = rotate(model, rotY, vec3(0, 1, 0));
        shade.set("model", model);
        view = lookAt(vec3(0, 0, zoom), vec3(0), vec3(0, 1, 0));
        proj = perspective(0.4f, 1.f, 0.1f, 100.f);
        shade.set("view", view);
        shade.set("projection", proj);
        shade.set("color", vec3(0., 0., 0.));
        ///====constructed mesh============
         glBindVertexArray(vao);
         shade.set("color", vec3(1));
         glDrawArrays(GL_TRIANGLES, 0, plist.size() / 3);
         ///========================
        // shade.set("color", vec3(0.0, 0.0, 0.0));
        // pcloud.draw(0,pcloud.posList.size());
         shade.unbind();
        glfwSwapBuffers(window);
    }
    glfwTerminate();
    return 0;
}
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (action == GLFW_PRESS) {
        if (key == GLFW_KEY_ESCAPE) {
            glfwSetWindowShouldClose(window, true);
        }
    }
}
void mouse_callback(GLFWwindow* window, int button, int action, int mods) {
    if (action == GLFW_PRESS) {
        mouse_press = true;
    }
    if (action == GLFW_RELEASE) {
        mouse_press = false;
        cout << rotY << " " << rotX << " " << zoom << endl;
    }
}
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
    zoom -= yoffset * 0.5;
}
void mouseInput(GLFWwindow* window) {
    double mX, mY;
    glfwGetCursorPos(window, &mX, &mY);
    mousex.push_back(mX / WIDTH * 2.0f - 1.0f);
    mousey.push_back(-(mY / HEIGHT * 2.0f - 1.0f));
    if (mousex.size() > 2) {
        mousex.erase(mousex.begin());
        mousey.erase(mousey.begin());
    }
    if (mouse_press) {
        rotY += (mousex[1] - mousex[0]) * 2;
        rotX += (mousey[1] - mousey[0]) * 2;
    }
}