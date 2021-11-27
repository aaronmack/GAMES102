//
// Created by 马西开 on 2020/12/3.
//
#include "minimal_surface.h"
bool judge_is_boundary(MyMesh::VertexHandle vh, MyMesh &mesh)
{
    bool flag = false;
    for(MyMesh::VertexEdgeIter ve_it = mesh.ve_iter(vh); ve_it.is_valid(); ++ve_it )
    {
        if(!ve_it->halfedge(0).is_valid())
        {
            flag = true;
            break;
        }
        if(!ve_it->halfedge(1).is_valid())
        {
            flag = true;
            break;
        }

    }
    return flag;

}


std::vector<MyMesh::Point> calc_Hn(MyMesh &mesh)
{
    std::vector<MyMesh::Point> ret(mesh.n_vertices());
    for(MyMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
    {
        double area = 0.0;
        //1-ring
        MyMesh::Point tp(0.0f,0.0f,0.0f);
        for(MyMesh::VertexOHalfedgeIter vh_it = mesh.voh_iter(*v_it); vh_it.is_valid(); ++vh_it)
        {
            double w = 0.0f;
            if(vh_it->is_valid())
            {
                //std::cout << calc_angle(*vh_it,mesh) << " ";
                w += 1.0f / tan(calc_angle(*vh_it,mesh)) ;
            }


            if(vh_it->opp().is_valid())
            {
                //std::cout << calc_angle(vh_it->opp(),mesh) << " "<<std::endl;
                w += 1.0f / tan(calc_angle(vh_it->opp(),mesh));
            }
            if(w < 0)
                w = 0;
            if(w > 10)
                w = 10;
            tp += w *(mesh.point(*v_it) - mesh.point(vh_it->to()));
            area += w * OpenMesh::sqrnorm(mesh.point(*v_it) - mesh.point(vh_it->to()));
        }
        area /= 8.0;
        tp /= (-4 * area);
        //std::cout << tp << std::endl;
        ret[v_it->idx()] = tp;

    }
    return ret;

}
MyMesh calc_minimal_surface(MyMesh mesh, float lambda, int it_num)
{
    /*mesh.update_face_normals();
    mesh.update_vertex_normals();*/
    for(int it = 0; it < it_num; ++it)
    {
        // calc Hn
        //std::vector<float> Mc = calc_Mean_Curvature(mesh);
        std::vector<MyMesh::Point> Hn = calc_Hn(mesh);
        // for all vertices
        for(MyMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
        {
            // fix boundary
            if(mesh.is_boundary(*v_it))
                continue;
            // update the vertex position
            mesh.point(*v_it) +=  lambda * Hn[v_it->idx()];
        }
        /*//update normals
        mesh.update_face_normals();
        mesh.update_vertex_normals();*/


    }

    /*mesh.update_face_normals();
    mesh.update_vertex_normals();*/
    return mesh;
}
MyMesh global_minimal_surface(MyMesh mesh)
{
    Eigen::SparseMatrix<float> A(mesh.n_vertices(),mesh.n_vertices());
    Eigen::SparseLU<Eigen::SparseMatrix<float>, Eigen::COLAMDOrdering<int> >solver;
    std::vector<float_tri> tv;
    Eigen::VectorXf b_x(mesh.n_vertices());
    Eigen::VectorXf b_y(mesh.n_vertices());
    Eigen::VectorXf b_z(mesh.n_vertices());
    //fill the coefficient matrix
    for(MyMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
    {
        //if is boundary, a_i = 1
        if(mesh.is_boundary(*v_it))
        {
            float_tri temp((*v_it).idx(),(*v_it).idx(),1.0f);
            tv.push_back(temp);
            b_x[(*v_it).idx()] = mesh.point(*v_it)[0];
            b_y[(*v_it).idx()] = mesh.point(*v_it)[1];
            b_z[(*v_it).idx()] = mesh.point(*v_it)[2];
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
            b_x[(*v_it).idx()] = 0.0f;
            b_y[(*v_it).idx()] = 0.0f;
            b_z[(*v_it).idx()] = 0.0f;
        }
    }
    //solve u,v
    A.setFromTriplets(tv.begin(),tv.end());
    solver.analyzePattern(A);
    solver.factorize(A);
    Eigen::VectorXf x = solver.solve(b_x);
    Eigen::VectorXf y = solver.solve(b_y);
    Eigen::VectorXf z = solver.solve(b_z);
    for(MyMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
    {
        if(!v_it->is_boundary())
        {
            int idx = (*v_it).idx();
            MyMesh::Point tp(x(idx),y(idx),z(idx));
            mesh.set_point(*v_it,tp);
        }

    }
    return mesh;
}