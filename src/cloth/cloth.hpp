#pragma once

#include "cgp/cgp.hpp"
#include "../environment.hpp"
#include <vector>

struct spring
{
    int id;
    float rest;
};

struct vertex_infos
{
    cgp::vec3 velocity = vec3(0, 0, 0);
    cgp::vec3 force = vec3(0, 0, 0);
    std::vector<spring> springs;
};

// Stores the buffers representing the cloth vertices
struct cloth_structure
{

    mesh shape;
    std::vector<vertex_infos> vertices;
    // Buffers are stored as 2D grid that can be accessed as grid(ku,kv)
    numarray<cgp::vec3> position;
    numarray<cgp::vec3> normal;

    // Also stores the triangle connectivity used to update the normals
    cgp::numarray<cgp::uint3> triangle_connectivity;

    // Update triangles having a vertice OldVerticeIndex and one in springsChanged
    void update_triangles(unsigned int oldVerticeIndex, unsigned int newVerticeIndex, std::vector<spring> springsChanged);

    // Update the specified triangle
    void update_triangle(unsigned int a, unsigned int newA, unsigned int b, unsigned int c);

    // Return whether if a vertex should break and ref a list of neighboors.
    bool should_break(int vertex, std::vector<int> &neighboorA, int remove);

    // Return the number of triangle with this couple
    int howMuchTriangles(unsigned int a, unsigned int b);

    // Return how much empty sides are surrounding the vertex
    int count_empty_side(int vertex, std::vector<int> &values);

    void initialize(int N_samples_edge, float len_border_cloth, float start_height_cloth); // Initialize a square flat cloth
    void update_normal();                                                                  // Call this function every time the cloth is updated before its draw
    int N_samples_edge() const;                                                            // Number of vertex along one dimension of the grid
};

// Helper structure and functions to draw a clothcgp::grid_2D<cgp::vec3> position;
// ********************************************** //
struct cloth_structure_drawable
{
    cgp::mesh_drawable drawable;

    void initialize(int N_sample_edge, float len_border_cloth, float start_height_cloth);
    void update(cloth_structure &cloth, cgp::opengl_texture_image_structure cloth_texture);
};

void draw(cloth_structure_drawable const &cloth_drawable, environment_generic_structure const &environment);
void draw_wireframe(cloth_structure_drawable const &cloth_drawable, environment_generic_structure const &environment);
void add_neighbor_vertex(std::vector<vertex_infos> *vertices, int vertex_a, int vertex_b, bool is_diag);
