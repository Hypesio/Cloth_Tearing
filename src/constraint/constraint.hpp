#pragma once

#include "cgp/cgp.hpp"
#include "../cloth/cloth.hpp"
#include "../gui.hpp"

// Parameters of the colliding sphere (center, radius)
struct sphere_parameter {
	cgp::vec3 center;
	float radius;
};

// Parameter attached to a fixed vertex (ku,kv) coordinates + 3D position
struct position_contraint {
	int i;
	cgp::vec3 position;
};


struct constraint_structure
{
	float ground_z = 0.0f;                                   // Height of the flood
	sphere_parameter sphere = { {0.1f, 0.5f, 0.0f}, 0.15f }; // Colliding sphere
	
	std::map<size_t, position_contraint> fixed_sample; // Storage of all fixed position of the cloth
	std::map<size_t, cgp::vec3> setA; 
	std::map<size_t, cgp::vec3> setB;

	// Will set the fixed point of the actual scene
	void set_fixed_point_scene(cloth_structure const& cloth, int scene_type, int n_sample, gui_parameters& gui);
	// Add a new fixed position
	void add_fixed_position(int i, cloth_structure const& cloth);
	// Remove a fixed position
	void remove_fixed_position(int i);
	// Update position of one constraints
	void update_position(int i, vec3 position);
	// Update positions of all constraints of set
	void update_positions(int set_index, vec3 position, int scene_type);
};