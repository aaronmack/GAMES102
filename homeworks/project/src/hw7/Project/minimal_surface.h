//
// Created by 马西开 on 2020/12/3.
//

#ifndef EXAMPLE_MINIMAL_SURFACE_H
#define EXAMPLE_MINIMAL_SURFACE_H

#include <igl/opengl/glfw/Viewer.h>
#include "Eigen/Sparse"
#include "Eigen/SparseLU"
#include "Base.hpp"
typedef Eigen::Triplet<float> float_tri;

// the function to judge boundary
// Attention: It's used for homework, you should use mesh.is_boundary() instead
// return true if it's boundary
bool judge_is_boundary(MyMesh::VertexHandle vh, MyMesh &mesh);
//get local area use barycentric cell
//calc the angle opposite the half edge
//return an eigen vector which vec[idx] is the mean curvature on the vertex idx
std::vector<MyMesh::Point> calc_Hn(MyMesh &mesh);


MyMesh calc_minimal_surface(MyMesh mesh, float lambda = 0.1, int it_num = 10);
MyMesh global_minimal_surface(MyMesh mesh);
#endif //EXAMPLE_MINIMAL_SURFACE_H
