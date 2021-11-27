#pragma once

#include <UGM/UGM.h>

struct CanvasData {
	Ubpa::valf2 scrolling{ 0.f,0.f };
	bool opt_enable_grid{ true };
	bool opt_enable_context_menu{ true };
	bool adding_line{ false };

	// Params for HW8
	bool show_sampling_points{ true };
	bool show_boundary{ true };
	bool run_single{ false };
	bool run_auto{ false };
	bool reset{ false };

	int width{ 540 };
	int height{ 340 };
	std::vector<Ubpa::pointf2> rectangle;
	std::vector<Ubpa::pointf2> sampling_points;
	std::vector<Ubpa::pointf2> sampling_points_origin;
	int num_sampling_points{ 10 };
	bool btn_random_sampling{ false };
	bool draw_delaunay{ false };
	bool draw_voronoi{ false };
	int num_lloyd_iterations{ 0 };
};

#include "details/CanvasData_AutoRefl.inl"
