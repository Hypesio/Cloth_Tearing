#pragma once

#include "cgp/cgp.hpp"
#include "../environment.hpp"
#include <vector>

struct vertex_infos {
    cgp::vec3 velocity;
    cgp::vec3 force; 
    std::vector<int> neightbors; 
    std::vector<float> neightbors_forces;
};

// Stores the buffers representing the cloth vertices
struct cloth_structure
{    

    std::vector<vertex_infos> vertices;
    // Buffers are stored as 2D grid that can be accessed as grid(ku,kv)
    
    numarray<cgp::vec3> position;  
    //std::vector<cgp::vec3> velocity;  
    //std::vector<cgp::vec3> force;
    numarray<cgp::vec3> normal;

    // Also stores the triangle connectivity used to update the normals
    cgp::numarray<cgp::uint3> triangle_connectivity;

    
    void initialize(int N_samples_edge, float len_border_cloth, float start_height_cloth);  // Initialize a square flat cloth
    void update_normal();       // Call this function every time the cloth is updated before its draw
    int N_samples_edge() const; // Number of vertex along one dimension of the grid
};


// Helper structure and functions to draw a clothcgp::grid_2D<cgp::vec3> position;
// ********************************************** //
struct cloth_structure_drawable
{
    cgp::mesh_drawable drawable;

    void initialize(int N_sample_edge, float len_border_cloth, float start_height_cloth);
    void update(cloth_structure const& cloth);
};

void draw(cloth_structure_drawable const& cloth_drawable, environment_generic_structure const& environment);
void draw_wireframe(cloth_structure_drawable const& cloth_drawable, environment_generic_structure const& environment);
