#include <OpenMesh/Core/IO/MeshIO.hh>
#include <igl/opengl/glfw/Viewer.h>
#include "Base.hpp"
#include "minimal_surface.h"
#include "Harmonic_map.hpp"



Eigen::MatrixXd V,tV;
Eigen::MatrixXi F,tF,F_uv;
Eigen::MatrixXd V_uv;
MyMesh tm, mesh;
Eigen::VectorXd mc,gc;

bool key_down(igl::opengl::glfw::Viewer& viewer, unsigned char key, int modifier)
{
    if (key == '1')
    {
        // Plot the 3D mesh
        tm = global_minimal_surface(mesh);
        openMesh_to_igl(tm,tV,tF);
        viewer.data().set_mesh(tV,tF);
        viewer.core().align_camera_center(tV,tF);
    }
    else if(key == '=')
    {
        tm = calc_minimal_surface(tm,0.001,30);
        openMesh_to_igl(tm,tV,tF);
        viewer.data().set_mesh(tV,tF);
        viewer.core().align_camera_center(tV,tF);
    }
    else if(key == '2')
    {
        //arc length circle
        MyMesh disk = getHarmonicMap(mesh, 0);
        openMesh_to_igl(disk,V_uv,F_uv);
        V_uv *= 5.0;
        viewer.data().clear();
        viewer.data().set_mesh(V,F);
        viewer.data().set_uv(V_uv);

        viewer.data().show_texture = true;
        viewer.core().align_camera_center(V,F);

    }
    else if(key == '3')
    {
        //uniform circle
        MyMesh disk = getHarmonicMap(mesh, 1);
        openMesh_to_igl(disk,V_uv,F_uv);
        V_uv *= 5.0;
        viewer.data().clear();
        viewer.data().set_mesh(V,F);
        viewer.data().set_uv(V_uv);

        viewer.data().show_texture = true;
        viewer.core().align_camera_center(V,F);
    }
    else if(key == '4')
    {
        //square
        MyMesh disk = getHarmonicMap(mesh, 2);
        openMesh_to_igl(disk,V_uv,F_uv);
        V_uv *= 10.0;
        viewer.data().clear();
        viewer.data().set_mesh(V,F);
        viewer.data().set_uv(V_uv);

        viewer.data().show_texture = true;
        viewer.core().align_camera_center(V,F);
    }
    else if(key == '5')
    {
        viewer.data().clear();
        viewer.data().set_mesh(V_uv,F);
        viewer.data().show_texture = true;
        viewer.core().align_camera_center(V_uv,F);

    }
    else if(key == '6')
    {
        viewer.data().show_texture = false;
    }
    viewer.data().compute_normals();

    return false;
}

int main(int argc, char *argv[])
{
    mesh.request_vertex_normals();
    mesh.request_face_normals();
    OpenMesh::IO::Options opt;
    if(!OpenMesh::IO::read_mesh(mesh,"../models/Nefertiti_face.obj",opt))
    {
        std::cout << "can't open file"<<std::endl;
    }
    /*for(MyMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
    {
        mesh.point(*v_it) *= 100.0;
    }*/
    mesh.update_face_normals();
    mesh.update_vertex_normals();
    tm = mesh;
    /*for(int i = 0; i < t.size(); ++ i)
    {
        std::cout << t[i] << std::endl;
}*/
    //MyMesh minimal_S = calc_minimal_surface(mesh,0.1,1);
    igl::opengl::glfw::Viewer viewer;
    viewer.callback_key_down = &key_down;
    V.setZero(mesh.n_vertices(),3);
    tV.setZero(mesh.n_vertices(),3);
    tF.setZero(mesh.n_faces(),3);
    F.setZero(mesh.n_faces(),3);
    openMesh_to_igl(mesh,V,F);
    viewer.data().set_mesh(V, F);
    MyMesh disk = getHarmonicMap(mesh);
    openMesh_to_igl(disk,V_uv,F_uv);
    V_uv *= 5.0;
    // Disable wireframe
    viewer.data().show_lines = false;
    viewer.data().show_texture = false;
    viewer.launch();

}
