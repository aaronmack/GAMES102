#include "CanvasSystem.h"
#include "../Components/CanvasData.h"
#include <_deps/imgui/imgui.h>
#include <spdlog/spdlog.h>

#include <cstring>
#include <cmath>
#include <numeric>
#include <ctime>

// includes for drawing the Voronoi Diagram
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Delaunay_triangulation_adaptation_traits_2.h>
#include <CGAL/Delaunay_triangulation_adaptation_policies_2.h>
#include <CGAL/Voronoi_diagram_2.h>

// typedefs for defining the adaptor
typedef CGAL::Exact_predicates_inexact_constructions_kernel                  K;
typedef CGAL::Delaunay_triangulation_2<K>                                    DT;
typedef CGAL::Delaunay_triangulation_adaptation_traits_2<DT>                 AT;
typedef CGAL::Delaunay_triangulation_caching_degeneracy_removal_policy_2<DT> AP;
typedef CGAL::Voronoi_diagram_2<DT, AT, AP>                                  VD;

// typedefs for the result type of the point location
typedef AT::Site_2                    Site_2;
typedef AT::Point_2                   Point_2;
typedef VD::Locate_result             Locate_result;
typedef VD::Face_handle               Face_handle;
typedef VD::Ccb_halfedge_circulator   Ccb_halfedge_circulator;
//typedef VD::Vertex_handle             Vertex_handle;
//typedef VD::Halfedge_handle           Halfedge_handle;

#define EPSILON 1E-6F

using namespace std;
using namespace Ubpa;

Ubpa::pointf2 operator+(const Ubpa::pointf2& a, const Ubpa::pointf2& b);
Ubpa::pointf2 operator-(const Ubpa::pointf2& a, const Ubpa::pointf2& b);
Ubpa::pointf2 operator*(const Ubpa::pointf2& a, const float b);
Ubpa::pointf2 operator/(const Ubpa::pointf2& a, const float b);
float CalDistance(pointf2 p1, pointf2 p2);
void DrawCurve(ImDrawList* draw_list, float origin_x, float origin_y, vector<pointf2> draw_points, ImU32 col, bool closed = false);
void Subdivide_Chaikin(vector<pointf2>& input_points);
void Subdivide_Cubic(vector<pointf2>& input_points);
bool IsPointOnLine(float px0, float py0, float px1, float py1, float px2, float py2);
bool IsIntersect(float px1, float py1, float px2, float py2, float px3, float py3, float px4, float py4);
bool IsPointInPolygon(float x, float y, const std::vector<Ubpa::pointf2>& P0);
pointf2 GetCrossPoint(pointf2 p1, pointf2 p2, pointf2 p3, pointf2 p4);
pointf2 GetCentroid(const vector<pointf2>& P);

void CanvasSystem::OnUpdate(Ubpa::UECS::Schedule& schedule) {
    schedule.RegisterCommand([](Ubpa::UECS::World* w) {
        auto data = w->entityMngr.GetSingleton<CanvasData>();
        if (!data)
            return;

        if (ImGui::Begin("Canvas")) {
            //ImGui::Checkbox("Enable grid", &data->opt_enable_grid);
            //ImGui::Checkbox("Enable context menu", &data->opt_enable_context_menu);
            ImGui::Checkbox("Show rect boundary", &data->show_boundary);
            ImGui::Checkbox("Show sampling points", &data->show_sampling_points);
            ImGui::Separator();

            ImGui::Text("Step 1: "); ImGui::SameLine();
            ImGui::PushItemWidth(100);
            ImGui::InputInt("width", &data->width);
            ImGui::PopItemWidth();
            ImGui::SameLine(250);
            ImGui::PushItemWidth(100);
            ImGui::InputInt("height", &data->height);
            ImGui::PopItemWidth();

            ImGui::Text("Step 2: "); ImGui::SameLine();
            data->btn_random_sampling = ImGui::Button("Random sampling");
            ImGui::SameLine(250);
            ImGui::PushItemWidth(100);
            ImGui::InputInt("sampling points", &data->num_sampling_points);
            ImGui::PopItemWidth();

            ImGui::Text("Step 3: "); ImGui::SameLine();
            ImGui::Checkbox("Delaunay\t", &data->draw_delaunay); ImGui::SameLine();
            ImGui::Checkbox("Voronoi", &data->draw_voronoi);

            ImGui::Text("Step 4: "); ImGui::SameLine();
            ImGui::Checkbox("run auto\t", &data->run_auto); ImGui::SameLine();
            data->run_single = ImGui::Button("run single"); ImGui::SameLine();
            ImGui::Text("\t"); ImGui::SameLine();
            data->reset = ImGui::Button("reset"); ImGui::SameLine();
            ImGui::Text("\t"); ImGui::SameLine();
            ImGui::Text("Lloyd Relaxation: %d", data->num_lloyd_iterations);

            ImGui::Text("Press [Z] to print the pos of all sampling points.");

            ImVec2 canvas_p0 = ImGui::GetCursorScreenPos();      // ImDrawList API uses screen coordinates!
            ImVec2 canvas_sz = ImGui::GetContentRegionAvail();   // Resize canvas to what's available
            if (canvas_sz.x < 50.0f) canvas_sz.x = 50.0f;
            if (canvas_sz.y < 50.0f) canvas_sz.y = 50.0f;
            ImVec2 canvas_p1 = ImVec2(canvas_p0.x + canvas_sz.x, canvas_p0.y + canvas_sz.y);

            // Draw border and background color
            ImGuiIO& io = ImGui::GetIO();
            ImDrawList* draw_list = ImGui::GetWindowDrawList();
            draw_list->AddRectFilled(canvas_p0, canvas_p1, IM_COL32(50, 50, 50, 255));
            draw_list->AddRect(canvas_p0, canvas_p1, IM_COL32(255, 255, 255, 255));

            // This will catch our interactions
            ImGui::InvisibleButton("canvas", canvas_sz, ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight);
            const bool is_hovered = ImGui::IsItemHovered(); // Hovered
            const bool is_active = ImGui::IsItemActive();   // Held
            const ImVec2 origin(canvas_p0.x + data->scrolling[0], canvas_p0.y + data->scrolling[1]); // Lock scrolled origin
            const pointf2 mouse_pos_in_canvas(io.MousePos.x - origin.x, io.MousePos.y - origin.y);

            // Step 1: Init rectangle region
            data->rectangle.clear();
            data->rectangle.push_back(pointf2(32.f, 32.f));
            data->rectangle.push_back(pointf2(32.f + data->width, 32.f));
            data->rectangle.push_back(pointf2(32.f + data->width, 32.f + data->height));
            data->rectangle.push_back(pointf2(32.f, 32.f + data->height));

            // Step 2: Random sampling
            if (data->btn_random_sampling) {
                [&]() {
                    data->sampling_points.clear();
                    srand((int)(time(NULL)));
                    for (int i = 0; i < data->num_sampling_points; i++) {
                        float x = (float)(32 + rand() % data->width);
                        float y = (float)(32 + rand() % data->height);
                        data->sampling_points.push_back(pointf2(x, y));
                    }
                    data->sampling_points_origin = vector(data->sampling_points);
                    data->num_lloyd_iterations = 0;
                }();
            }

            if (data->reset && !data->sampling_points_origin.empty()) {
                data->sampling_points = vector(data->sampling_points_origin);
                data->num_lloyd_iterations = 0;
            }

            // Step 3: Draw delaunay & voronoi graph
            if (data->sampling_points.size() >= 6) {
                VD vd;
                for (auto siter = data->sampling_points.begin(); siter != data->sampling_points.end(); siter++)
                    vd.insert(Site_2(siter->at(0), siter->at(1)));
                DT dt = vd.dual();

                // ��һ��DT��ʼ������
                /*DT dt;
                vector<K::Point_2> data_sampling_points;
                for (auto siter = data->sampling_points.begin(); siter != data->sampling_points.end(); siter++)
                    data_sampling_points.push_back(K::Point_2(siter->at(0), siter->at(1)));
                dt.insert(data_sampling_points.begin(), data_sampling_points.end());*/

                // a. draw delaunay triangle
                for (auto fiter = dt.finite_faces_begin(); fiter != dt.finite_faces_end(); fiter++) {
                    vector<pointf2> delaunay_triangle;
                    for (int i = 0; i < 3; i++)
                        delaunay_triangle.push_back(pointf2(fiter->vertex(i)->point().x(), fiter->vertex(i)->point().y()));

                    if (data->draw_delaunay)
                        DrawCurve(draw_list, origin.x, origin.y, delaunay_triangle, IM_COL32(255, 235, 205, 255), true);
                }

                // b. draw voronoi graph
                // ��DTҲ��ֱ�ӻ�Voronoiͼ�����������Ҷ�Ӧ������
                /*for (auto eiter = dt.edges_begin(); eiter != dt.edges_end(); eiter++) {
                    // Delaunay��żͼ�ı�
                    auto dual_edge = dt.dual(eiter);
                    auto dual_edge_seg = CGAL::object_cast<K::Segment_2>(&dual_edge);
                    auto dual_edge_ray = CGAL::object_cast<K::Ray_2>(&dual_edge);
                    if (dual_edge_seg) {
                        // �ñ����߶�
                        pointf2 p1((float)dual_edge_seg->source().hx(), (float)dual_edge_seg->source().hy()), p2((float)dual_edge_seg->target().hx(), (float)dual_edge_seg->target().hy());

                        //for (auto viter = data->type_points.begin(); viter != data->type_points.end(); viter++) {
                            //auto viter_next = (viter != data->type_points.end() - 1) ? viter + 1 : data->type_points.begin();
                            //pointf2 p3(viter->at(0), viter->at(1)), p4(viter_next->at(0), viter_next->at(1));
                            //if (IsIntersect(p1[0], p1[1], p2[0], p2[1], p3[0], p3[1], p4[0], p4[1])) {
                                //GetCrossPoint(p1, p2, p3, p4);
                            //}
                        //}

                        draw_list->AddLine(ImVec2(origin.x + p1[0], origin.y + p1[1]), ImVec2(origin.x + p2[0], origin.y + p2[1]), IM_COL32(88, 87, 86, 255), 2.0f);
                    }
                    else if (dual_edge_ray) {
                        // �ñ�������
                        int index = 1;
                        while (IsPointInPolygon((float)dual_edge_ray->point(index).hx(), (float)dual_edge_ray->point(index).hy(), data->rectangle))
                            index++;
                        pointf2 p1((float)dual_edge_ray->source().hx(), (float)dual_edge_ray->source().hy()), p2((float)dual_edge_ray->point(index).hx(), (float)dual_edge_ray->point(index).hy());
                        if (IsPointInPolygon(p1[0], p1[1], data->rectangle))
                            draw_list->AddLine(ImVec2(origin.x + p1[0], origin.y + p1[1]), ImVec2(origin.x + p2[0], origin.y + p2[1]), IM_COL32(88, 87, 86, 255), 2.0f);
                    }
                }*/

                for (auto siter = data->sampling_points.begin(); siter != data->sampling_points.end(); siter++) {
                    Locate_result lr = vd.locate(Point_2(siter->at(0), siter->at(1)));
                    Face_handle* f = boost::get<Face_handle>(&lr);
                    //if ((*f)->is_unbounded()) continue;

                    // ��ǰ������(*siter)��Ӧ��voronoi��������򣬽�����ζ��㣨������4���㣩������voronoi_polygon�У�����Lloyd Relaxation
                    vector<pointf2> voronoi_polygon;
                    Ccb_halfedge_circulator ec = (*f)->ccb();
                    do {
                        pointf2 sp, tp;
                        if (ec->has_source())
                            sp = pointf2(ec->source()->point().hx(), ec->source()->point().hy());
                        else {
                            // �����(*ec)Ϊ����ʱ���Ƚ�vdת����DT��ʽ������dt.dual(ec->dual())���Ray_2��ʽ����ray
                            // �ο�http://cgal-discuss.949826.n4.nabble.com/Voronoi-unbounded-halfedge-to-ray-2-td4662076.html ��5¥
                            auto obj = vd.dual().dual(ec->opposite()->dual());
                            auto ray = CGAL::object_cast<K::Ray_2>(&obj);

                            int index = 1;
                            while (IsPointInPolygon((float)ray->point(index).hx(), (float)ray->point(index).hy(), data->rectangle))
                                index++;
                            sp = pointf2(ray->point(index).hx(), ray->point(index).hy());
                        }

                        if (ec->has_target())
                            tp = pointf2(ec->target()->point().hx(), ec->target()->point().hy());
                        else {
                            // ����ͬ��
                            auto obj = vd.dual().dual(ec->dual());
                            auto ray = CGAL::object_cast<K::Ray_2>(&obj);

                            int index = 1;
                            while (IsPointInPolygon((float)ray->point(index).hx(), (float)ray->point(index).hy(), data->rectangle))
                                index++;
                            tp = pointf2(ray->point(index).hx(), ray->point(index).hy());
                        }

                        bool isSpInside = IsPointInPolygon(sp[0], sp[1], data->rectangle);
                        bool isTpInside = IsPointInPolygon(tp[0], tp[1], data->rectangle);

                        // �洢λ�������ڵ�source_point
                        if (isSpInside)
                            voronoi_polygon.push_back(sp);

                        // �洢voronoi������α߽��ཻ�ĵ㣬�Լ�����4����
                        for (auto riter = data->rectangle.begin(); riter != data->rectangle.end(); riter++) {
                            auto riter_next = (riter != data->rectangle.end() - 1) ? riter + 1 : data->rectangle.begin();
                            if (IsIntersect(sp[0], sp[1], tp[0], tp[1], riter->at(0), riter->at(1), riter_next->at(0), riter_next->at(1))) {
                                pointf2 cp = GetCrossPoint(sp, tp, *riter, *riter_next);
                                voronoi_polygon.push_back(cp);
                            }
                        }

                        if (!isTpInside) {
                            // �жϲ����Ӿ��ζ���
                            for (auto riter = data->rectangle.begin(); riter != data->rectangle.end(); riter++) {
                                Locate_result lr_r = vd.locate(Point_2(riter->at(0), riter->at(1)));
                                Face_handle* f_r = boost::get<Face_handle>(&lr_r);
                                if (*f_r == *f) {
                                    voronoi_polygon.push_back(pointf2(riter->at(0), riter->at(1)));
                                    break;
                                }
                            }
                        }
                    } while (++ec != (*f)->ccb());

                    // Step 4: Lloyd algorithm
                    // ��Voronoi����ε����ģ�����������
                    if (data->run_single || data->run_auto)
                        *siter = GetCentroid(voronoi_polygon);

                    if (data->draw_voronoi && voronoi_polygon.size() > 1)
                        DrawCurve(draw_list, origin.x, origin.y, voronoi_polygon, IM_COL32(202, 235, 216, 255), true);
                }
                if (data->run_single || data->run_auto) data->num_lloyd_iterations++;
            }

            // Pan (we use p zero mouse threshold when there's no context menu)
            // You may decide to make that threshold dynamic based on whether the mouse is hovering something etc.
            const float mouse_threshold_for_pan = data->opt_enable_context_menu ? -1.0f : 0.0f;
            if (is_active && ImGui::IsMouseDragging(ImGuiMouseButton_Right, mouse_threshold_for_pan))
            {
                data->scrolling[0] += io.MouseDelta.x;
                data->scrolling[1] += io.MouseDelta.y;
            }

            // Context menu (under default mouse threshold)
            ImVec2 drag_delta = ImGui::GetMouseDragDelta(ImGuiMouseButton_Right);
            if (data->opt_enable_context_menu && ImGui::IsMouseReleased(ImGuiMouseButton_Right) && drag_delta.x == 0.0f && drag_delta.y == 0.0f)
                ImGui::OpenPopupContextItem("context");
            if (ImGui::BeginPopup("context")) {
                if (ImGui::MenuItem("Remove all", NULL, false, data->sampling_points.size() > 0)) {
                    data->sampling_points.clear();
                    data->sampling_points_origin.clear();
                    data->num_lloyd_iterations = 0;
                }
                ImGui::EndPopup();
            }

            // Print all coordinates
            if (ImGui::IsKeyPressed(ImGui::GetKeyIndex(ImGuiKey_Z)) && !data->sampling_points.empty()) {
                for (int i = 0; i < data->sampling_points.size(); i++) {
                    spdlog::info("(" + to_string((int)data->sampling_points[i][0]) + ", " + to_string((int)data->sampling_points[i][1]) + ")");
                }
            }

            // Draw grid + all lines in the canvas
            draw_list->PushClipRect(canvas_p0, canvas_p1, true);
            if (data->opt_enable_grid)
            {
                const float GRID_STEP = 64.0f;
                for (float x = fmodf(data->scrolling[0], GRID_STEP); x < canvas_sz.x; x += GRID_STEP)
                    draw_list->AddLine(ImVec2(canvas_p0.x + x, canvas_p0.y), ImVec2(canvas_p0.x + x, canvas_p1.y), IM_COL32(200, 200, 200, 40));
                for (float y = fmodf(data->scrolling[1], GRID_STEP); y < canvas_sz.y; y += GRID_STEP)
                    draw_list->AddLine(ImVec2(canvas_p0.x, canvas_p0.y + y), ImVec2(canvas_p1.x, canvas_p0.y + y), IM_COL32(200, 200, 200, 40));
            }

            // Draw curve & sampling points
            // Draw rectangle
            if (data->show_boundary)
                DrawCurve(draw_list, origin.x, origin.y, data->rectangle, IM_COL32(0, 255, 255, 255), true);

            // Draw sampling points
            if (data->show_sampling_points && !data->sampling_points.empty()) {
                for (int i = 0; i < data->sampling_points.size(); i++) {
                    draw_list->AddCircleFilled(ImVec2(data->sampling_points[i][0] + origin.x, data->sampling_points[i][1] + origin.y), 3.0f, IM_COL32(255, 0, 0, 255));
                }
            }

            draw_list->PopClipRect();
        }

        ImGui::End();
    });
}

Ubpa::pointf2 operator+(const Ubpa::pointf2& p1, const Ubpa::pointf2& p2) {
    return Ubpa::pointf2(p1[0] + p2[0], p1[1] + p2[1]);
}

Ubpa::pointf2 operator-(const Ubpa::pointf2& p1, const Ubpa::pointf2& p2) {
    return Ubpa::pointf2(p1[0] - p2[0], p1[1] - p2[1]);
}

Ubpa::pointf2 operator*(const Ubpa::pointf2& p, const float f) {
    return Ubpa::pointf2(p[0] * f, p[1] * f);
}

Ubpa::pointf2 operator/(const Ubpa::pointf2& p, const float f) {
    assert(f > EPSILON);
    return Ubpa::pointf2(p[0] / f, p[1] / f);
}

float CalDistance(pointf2 p1, pointf2 p2) {
    return sqrt(pow(p1[0] - p2[0], 2) + pow(p1[1] - p2[1], 2));
}

void DrawCurve(ImDrawList* draw_list, float origin_x, float origin_y, vector<pointf2> draw_points, ImU32 col, bool closed) {
    for (int i = 0; i < draw_points.size() - 1; i++) {
        draw_list->AddLine(ImVec2(origin_x + draw_points.at(i).at(0), origin_y + draw_points.at(i).at(1)), ImVec2(origin_x + draw_points.at(i + 1).at(0), origin_y + draw_points.at(i + 1).at(1)), col, 2.0f);
    }
    if (closed)
        draw_list->AddLine(ImVec2(origin_x + draw_points[0][0], origin_y + draw_points[0][1]), ImVec2(origin_x + draw_points.back()[0], origin_y + draw_points.back()[1]), col, 2.0f);
}

void Subdivide_Chaikin(vector<pointf2>& input_points) {
    vector<pointf2> output_points(input_points.size() * 2);

    for (int i = 0; i < input_points.size(); i++) {
        if (i > 0) {
            output_points[2 * i][0] = 0.25f * input_points[i - 1][0] + 0.75f * input_points[i][0];
            output_points[2 * i][1] = 0.25f * input_points[i - 1][1] + 0.75f * input_points[i][1];
        }
        else
        {
            output_points[2 * i][0] = 0.25f * input_points[input_points.size() - 1][0] + 0.75f * input_points[i][0];
            output_points[2 * i][1] = 0.25f * input_points[input_points.size() - 1][1] + 0.75f * input_points[i][1];
        }
        if (i < input_points.size() - 1) {
            output_points[2 * i + 1][0] = 0.75f * input_points[i][0] + 0.25f * input_points[i + 1][0];
            output_points[2 * i + 1][1] = 0.75f * input_points[i][1] + 0.25f * input_points[i + 1][1];
        }
        else {
            output_points[2 * i + 1][0] = 0.75f * input_points[i][0] + 0.25f * input_points[0][0];
            output_points[2 * i + 1][1] = 0.75f * input_points[i][1] + 0.25f * input_points[0][1];
        }
    }

    input_points = vector(output_points);
}

void Subdivide_Cubic(vector<pointf2>& input_points) {
    vector<pointf2> output_points(input_points.size() * 2);

    for (int i = 0; i < input_points.size(); i++) {
        if (i > 0 && i < input_points.size() - 1) {
            output_points[2 * i][0] = 0.125f * input_points[i - 1][0] + 0.75f * input_points[i][0] + 0.125f * input_points[i + 1][0];
            output_points[2 * i][1] = 0.125f * input_points[i - 1][1] + 0.75f * input_points[i][1] + 0.125f * input_points[i + 1][1];
        }
        else if (i == 0)
        {
            output_points[2 * i][0] = 0.125f * input_points[input_points.size() - 1][0] + 0.75f * input_points[i][0] + 0.125f * input_points[i + 1][0];
            output_points[2 * i][1] = 0.125f * input_points[input_points.size() - 1][1] + 0.75f * input_points[i][1] + 0.125f * input_points[i + 1][1];
        }
        else {
            output_points[2 * i][0] = 0.125f * input_points[i - 1][0] + 0.75f * input_points[i][0] + 0.125f * input_points[0][0];
            output_points[2 * i][1] = 0.125f * input_points[i - 1][1] + 0.75f * input_points[i][1] + 0.125f * input_points[0][1];
        }
        if (i < input_points.size() - 1) {
            output_points[2 * i + 1][0] = 0.5f * input_points[i][0] + 0.5f * input_points[i + 1][0];
            output_points[2 * i + 1][1] = 0.5f * input_points[i][1] + 0.5f * input_points[i + 1][1];
        }
        else {
            output_points[2 * i + 1][0] = 0.5f * input_points[i][0] + 0.5f * input_points[0][0];
            output_points[2 * i + 1][1] = 0.5f * input_points[i][1] + 0.5f * input_points[0][1];
        }
    }

    input_points = vector(output_points);
}

// ���߷��жϲ������Ƿ��ڶ�����ڲ�
bool IsPointOnLine(float px0, float py0, float px1, float py1, float px2, float py2) {
    // �жϵ����߶���
    bool flag = false;
    float d1 = (px1 - px0) * (py2 - py0) - (px2 - px0) * (py1 - py0);
    if ((fabs(d1) < EPSILON) && ((px0 - px1) * (px0 - px2) <= -EPSILON) && ((py0 - py1) * (py0 - py2) <= -EPSILON))
        flag = true;

    return flag;
}

bool IsIntersect(float px1, float py1, float px2, float py2, float px3, float py3, float px4, float py4) {
    // �ж����߶��ཻ
    bool flag = false;
    float d = (px2 - px1) * (py4 - py3) - (py2 - py1) * (px4 - px3);
    if (fabs(d) > EPSILON) {
        float r = ((py1 - py3) * (px4 - px3) - (px1 - px3) * (py4 - py3)) / d;
        float s = ((py1 - py3) * (px2 - px1) - (px1 - px3) * (py2 - py1)) / d;
        if ((r >= EPSILON) && (r <= 1.f) && (s >= EPSILON) && (s <= 1.f))
            flag = true;
    }
    return flag;
}

bool IsPointInPolygon(float x, float y, const std::vector<Ubpa::pointf2>& P) {
    // �жϵ��ڶ������
    vector<pointf2> P1(P);
    P1.push_back(P1[0]);
    bool flag = false;
    int count = 0;

    float minX = FLT_MAX;
    for (int i = 0; i < P1.size(); i++)
        minX = std::min(minX, P1[i][0]);

    float px = x;
    float py = y;
    float linePoint1x = x;
    float linePoint1y = y;
    float linePoint2x = minX - 10.0f; // ȡ��С��Xֵ��С��ֵ��Ϊ���ߵ��յ�
    float linePoint2y = y;

    // ����ÿһ����
    for (int i = 0; i < P1.size() - 1; i++) {
        float cx1 = P1[i][0];
        float cy1 = P1[i][1];
        float cx2 = P1[i + 1][0];
        float cy2 = P1[i + 1][1];

        if (IsPointOnLine(px, py, cx1, cy1, cx2, cy2))
            return false;

        if (fabs(cy2 - cy1) < EPSILON) // ƽ�����ཻ
            continue;

        if (IsPointOnLine(cx1, cy1, linePoint1x, linePoint1y, linePoint2x, linePoint2y) && cy1 > cy2)
            // ֻ��֤�϶˵�+1
            count++;
        else if (IsPointOnLine(cx2, cy2, linePoint1x, linePoint1y, linePoint2x, linePoint2y) && cy2 > cy1)
            // ֻ��֤�϶˵�+1
            count++;
        else if (IsIntersect(cx1, cy1, cx2, cy2, linePoint1x, linePoint1y, linePoint2x, linePoint2y))
            // ���ų�ƽ�е����
            count++;
    }

    if (count % 2 == 1)
        flag = true;

    return flag;
}

// �������߶ν���
pointf2 GetCrossPoint(pointf2 p1, pointf2 p2, pointf2 p3, pointf2 p4) {
    assert(IsIntersect(p1[0], p1[1], p2[0], p2[1], p3[0], p3[1], p4[0], p4[1]));
    valf2 base = p4 - p3;
    float d1 = fabs(base[0] * (p1 - p3)[1] - base[1] * (p1 - p3)[0]);
    float d2 = fabs(base[0] * (p2 - p3)[1] - base[1] * (p2 - p3)[0]);
    float t = d1 / (d1 + d2);
    return p1 + (p2 - p1) * t;
}

// �������ε�����
pointf2 GetCentroid(const vector<pointf2>& P) {
    float area = 0.f;
    pointf2 c{ 0.f };
    int size = P.size();

    for (int i = 0; i < size - 1; i++) {
        area += (P[i][0] * P[i + 1][1] - P[i + 1][0] * P[i][1]) / 2.f;
        c[0] += (P[i][0] * P[i + 1][1] - P[i + 1][0] * P[i][1]) * (P[i][0] + P[i + 1][0]);
        c[1] += (P[i][0] * P[i + 1][1] - P[i + 1][0] * P[i][1]) * (P[i][1] + P[i + 1][1]);
    }

    area += (P[size - 1][0] * P[0][1] - P[0][0] * P[size - 1][1]) / 2.f;
    c[0] += (P[size - 1][0] * P[0][1] - P[0][0] * P[size - 1][1]) * (P[size - 1][0] + P[0][0]);
    c[1] += (P[size - 1][0] * P[0][1] - P[0][0] * P[size - 1][1]) * (P[size - 1][1] + P[0][1]);

    c[0] /= 6.f * area;
    c[1] /= 6.f * area;

    return c;
}
