#include <OpenMesh/Core/IO/MeshIO.hh>
#include <igl/opengl/glfw/Viewer.h>
#include "Base.hpp"
#include "SurfaceSimplification.h"


const int cache_size = 12;
Eigen::MatrixXd V[cache_size];
Eigen::MatrixXi F[cache_size];
bool flag[cache_size] = {false};
MyMesh mesh;
int ti = 1;
bool key_down(igl::opengl::glfw::Viewer& viewer, unsigned char key, int modifier)
{
    if(key == '1')
    {
        if(ti < cache_size)
        {
            if(!flag[ti])
            {
                Surface_Simplification(mesh,0.5);
                std::cout << "vertices : " << mesh.n_vertices() << std::endl;;
                std::cout << "faces : " << mesh.n_faces() << std::endl;;
                openMesh_to_igl(mesh,V[ti],F[ti]);
                flag[ti] = true;
            }
            viewer.data().clear();
            viewer.data().set_mesh(V[ti],F[ti]);
            viewer.core().align_camera_center(V[ti],F[ti]);
            ti++;
        }

    }
    else if(key == '2')
    {
        if(ti > 1)
        {
            ti--;
            viewer.data().clear();
            viewer.data().set_mesh(V[ti],F[ti]);
            viewer.core().align_camera_center(V[ti],F[ti]);
        }
    }
    return false;
}

int main(int argc, char *argv[])
{

    OpenMesh::VPropHandleT< double > test;
    mesh.add_property(test);
    mesh.request_vertex_normals();
    mesh.request_face_normals();
    OpenMesh::IO::Options opt;
    if(!OpenMesh::IO::read_mesh(mesh,"../models/squirrel.obj",opt))
    {
        std::cout << "can't open file"<<std::endl;
    }
    for(MyMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
    {
        mesh.point(*v_it) /= 4.0;
    }
    mesh.update_face_normals();
    mesh.update_vertex_normals();
    igl::opengl::glfw::Viewer viewer;
    viewer.callback_key_down = &key_down;
    openMesh_to_igl(mesh,V[0],F[0]);

    std::cout << "vertices : " <<  mesh.n_vertices() << std::endl;
    std::cout << "faces : " << mesh.n_faces() << std::endl;
    viewer.data().set_mesh(V[0], F[0]);
    // Disable wireframe
    viewer.data().show_lines = false;
    //viewer.data().show_texture = true;
    viewer.launch();

}
