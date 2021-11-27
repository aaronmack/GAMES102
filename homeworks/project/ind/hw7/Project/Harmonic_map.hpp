//
// Created by 马西开 on 2020/12/2.
//

#ifndef EXAMPLE_HARMONIC_MAP_HPP
#define EXAMPLE_HARMONIC_MAP_HPP

#include "Eigen/Sparse"
#include "Eigen/SparseLU"
#include "Eigen/SparseQR"
#include "Base.hpp"
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <iostream>
typedef OpenMesh::TriMesh_ArrayKernelT<> MyMesh;
typedef Eigen::Triplet<float> float_tri;
//const float R = 1.0f;

void circle_init(MyMesh &mesh, std::vector<MyMesh::VertexHandle>& boundary, Eigen::VectorXf& b_u, Eigen::VectorXf& b_v, float R)
{
    float delta_angle = 2.0f * M_PI / static_cast<float>(boundary.size());
    for(size_t i = 0; i < boundary.size(); ++i)
    {
        b_u[boundary[i].idx()] = R * cos(i * delta_angle);
        b_v[boundary[i].idx()] = R * sin(i * delta_angle);
    }
}

void arc_length_Circle_init(MyMesh &mesh, std::vector<MyMesh::VertexHandle>& boundary, Eigen::VectorXf& b_u, Eigen::VectorXf& b_v, float R)
{
    float arc_len = 0.0f;
    for(int i = 1; i < boundary.size(); ++i)
    {
        arc_len += OpenMesh::norm(mesh.point(boundary[i]) - mesh.point(boundary[i-1]));
    }
    arc_len += OpenMesh::norm(mesh.point(boundary[boundary.size()-1]) - mesh.point(boundary[0]));
    std::vector<float> delta;
    for(int i = 1; i < boundary.size(); ++i)
    {
        float t_len = OpenMesh::norm(mesh.point(boundary[i]) - mesh.point(boundary[i-1]));
        delta.push_back(2.0f * M_PI * (t_len / arc_len));
    }
    // ERROR on windows (Index out of range) xxxx
    float angle_now = 0.0f;
    std::cout << "bs: " << boundary.size() << "\n";
    for(size_t i = 0; i < boundary.size(); ++i)
    {
        b_u[boundary[i].idx()] = R * cos(angle_now);
        b_v[boundary[i].idx()] = R * sin(angle_now);
        if (i == boundary.size() - 1)  // xxxx
            continue;
        angle_now += delta[i];
    }
}
void square_init(MyMesh &mesh, std::vector<MyMesh::VertexHandle>& boundary, Eigen::VectorXf& b_u, Eigen::VectorXf& b_v, float length)
{
    int delta_size = boundary.size() / 4;
    int landmark_idx1 = delta_size;
    int landmark_idx2 = 2 * delta_size;
    int landmark_idx3 = 3 * delta_size;
    b_u[boundary[0].idx()] = 0.0f;
    b_v[boundary[0].idx()] = 0.0f;
    float delta = length / landmark_idx1;
    for (int i = 0; i < landmark_idx1; ++i) {
        b_u[boundary[i].idx()] = 0.0f;
        b_v[boundary[i].idx()] = i * delta;

    }
    b_u[boundary[landmark_idx1].idx()] = 0.0f;
    b_v[boundary[landmark_idx1].idx()] = length;
    delta = length / (landmark_idx2 - landmark_idx1);
    for(int i = landmark_idx1 + 1,j = 0; i < landmark_idx2; ++i,++j)
    {
        b_u[boundary[i].idx()] = j * delta;
        b_v[boundary[i].idx()] = length;

    }
    b_u[boundary[landmark_idx2].idx()] = length;
    b_v[boundary[landmark_idx2].idx()] = length;
    delta = length / (landmark_idx3 - landmark_idx2);
    for(int i = landmark_idx2 + 1,j = 0; i < landmark_idx3; ++i,++j)
    {
        b_u[boundary[i].idx()] = length;
        b_v[boundary[i].idx()] = length - j * delta;

    }
    b_u[boundary[landmark_idx3].idx()] = length;
    b_v[boundary[landmark_idx3].idx()] = 0;
    delta = length / (boundary.size() - landmark_idx3);
    for(int i = landmark_idx3 + 1,j = 0; i < boundary.size(); ++i,++j)
    {
        b_u[boundary[i].idx()] = length - j * delta;
        b_v[boundary[i].idx()] = 0;

    }
}
MyMesh getHarmonicMap(MyMesh &mesh, int mode = 2)
{
    MyMesh disk(mesh);
    //find a boundary vertex
    MyMesh::VertexHandle start_v;
    bool flag = false;
    for(MyMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
    {
        if(mesh.is_boundary(*v_it))
        {
            start_v = *v_it;
            flag = true;
            break;
        }
    }
    if(!flag)
    {
        std::cerr<<"No boundary!"<<std::endl;
        return disk;
    }
    //get all boundary vertices
    std::vector<MyMesh::VertexHandle> boundary;
    MyMesh::VertexHandle pre,now;
    boundary.push_back(start_v);
    now = start_v;
    for(MyMesh::VertexVertexIter vv_it = mesh.vv_iter(start_v); vv_it.is_valid(); ++vv_it)
    {
        if(mesh.is_boundary(*vv_it))
        {
            pre = now;
            now = *vv_it;
            break;
        }
    }
    while(now != start_v)
    {
        for(MyMesh::VertexVertexIter vv_it = mesh.vv_iter(now); vv_it.is_valid(); ++vv_it)
        {
            if(mesh.is_boundary(*vv_it) && *vv_it != pre)
            {
                pre = now;
                boundary.push_back(pre);
                now = *vv_it;
                break;
            }
        }
    }

    Eigen::VectorXf b_u(mesh.n_vertices());
    Eigen::VectorXf b_v(mesh.n_vertices());
    if(mode == 0)
        arc_length_Circle_init(mesh,  boundary,  b_u,  b_v, 1.0f);
    else if(mode == 2)
        square_init(mesh,  boundary,  b_u,  b_v, 1.0f);
    else
        circle_init(mesh,  boundary,  b_u,  b_v, 1.0f);
    Eigen::SparseMatrix<float> A(mesh.n_vertices(),mesh.n_vertices());
    Eigen::SparseLU<Eigen::SparseMatrix<float>, Eigen::COLAMDOrdering<int> >solver;
    std::vector<float_tri> tv;
    //fill the coefficient matrix
    for(MyMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
    {
        //if is boundary, a_i = 1
        if(mesh.is_boundary(*v_it))
        {
            float_tri temp((*v_it).idx(),(*v_it).idx(),1.0f);
            tv.push_back(temp);
        }
            // if is not boundary
        else
        {
            float cont = 0;
            //negative number of the sum of the 1-ring coefficient
            for(MyMesh::VertexOHalfedgeIter vh_it = mesh.voh_iter(*v_it); vh_it.is_valid(); ++vh_it)
            {
                //calc cot weight
                float w1 = 1.0f / tanf(calc_angle(*vh_it,mesh));
                float w2 = 1.0f / tanf(calc_angle((*vh_it).opp(),mesh));
                float w = w1 + w2;
                float_tri temp((*v_it).idx(),(*vh_it).to().idx(), w);
                tv.push_back(temp);
                cont += w;

            }
            float_tri temp((*v_it).idx(),(*v_it).idx(),-cont);
            tv.push_back(temp);
            b_u[(*v_it).idx()] = 0.0f;
            b_v[(*v_it).idx()] = 0.0f;
        }
    }
    //solve u,v
    A.setFromTriplets(tv.begin(),tv.end());
    solver.analyzePattern(A);
    solver.factorize(A);
    Eigen::VectorXf disk_u = solver.solve(b_u);
    Eigen::VectorXf disk_v = solver.solve(b_v);
    //set up parameterized mesh
    for(MyMesh::VertexIter v_it = disk.vertices_begin(); v_it != disk.vertices_end(); ++v_it)
    {
        int idx = (*v_it).idx();
        float u = disk_u(idx);
        float v = disk_v(idx);
        MyMesh::Point tp(u,v,0.0f);
        disk.set_point(*v_it,tp);
    }
    return disk;



}

#endif //EXAMPLE_HARMONIC_MAP_HPP
