#include "simulation.hpp"

#include <iostream>
#include <algorithm>

using namespace cgp;

// Compute all the forces applied on each vertex
void simulation_compute_force(cloth_structure &cloth, simulation_parameters const &parameters)
{
    std::vector<vertex_infos> &vertices = cloth.vertices;
    numarray<vec3> const &position = cloth.position;
    numarray<vec3> const &normal = cloth.normal;

    size_t const N = vertices.size();

    float const K = parameters.K;
    float const m = parameters.mass_total / N;
    float const mu = parameters.mu;

    const vec3 g = {0, 0, -9.81f};
    for (vertex_infos &v : vertices)
    {
        v.force = m * g - mu * m * v.velocity; // Gravity + Drag Force
    }

    for (int i = 0; i < N; ++i)
    {
        vertex_infos &v = vertices[i];
        for (spring s : v.springs)
        {
            vec3 vn = position[s.id] - position[i];
            if (norm(vn) == 0)
                continue;
            v.force += K * (norm(vn) - s.rest) * normalize(vn); // Spring Force
        }
    }

    if (parameters.wind.magnitude)
    {
        vec3 wind = parameters.wind.magnitude * parameters.wind.direction;
        for (int i = 0; i < N; ++i)
        {
            vertices[i].force += dot(-wind, normal[i]) * normal[i]; // Wind Force
        }
    }
}

// Apply forces and velocities on positions
void simulation_numerical_integration(cloth_structure &cloth, simulation_parameters const &parameters, float dt)
{
    std::vector<vertex_infos> &vertices = cloth.vertices;
    numarray<vec3> &position = cloth.position;

    size_t const N = vertices.size();
    float const m = parameters.mass_total / static_cast<float>(N);
    
    for (int i = 0; i < N; ++i)
    {
        vertex_infos &v = vertices[i];
        v.velocity += (dt * v.force / m);
        position[i] += (dt * v.velocity);
    }
}

// Apply everything that will constraint a point's position
void simulation_apply_constraints(cloth_structure &cloth, constraint_structure const &constraint)
{
    std::vector<vertex_infos> &vertices = cloth.vertices;
    numarray<vec3> &position = cloth.position;

    size_t const N = vertices.size();

    float const floor_height = .001;
    float const sphere_collision = .02;

    for (auto const &it : constraint.fixed_sample)
    {
        position_contraint c = it.second;
        position[c.i] = c.position;
    }

    for (int i = 0; i < N; i++)
    {
        if (position[i].z < constraint.ground_z + floor_height)
        {
            position[i].z = constraint.ground_z + floor_height;
            vertices[i].velocity.z = 0;
        }

        vec3 sp = position[i] - constraint.sphere.center;
        if (norm(sp) < constraint.sphere.radius + sphere_collision)
        {
            vertex_infos &v = vertices[i];
            vec3 normal = normalize(sp);

            position[i] = constraint.sphere.center + normalize(sp) * (constraint.sphere.radius + sphere_collision);
            v.velocity -= dot(v.velocity, normal) * normal;
        }
    }
}

inline mat3 get_symetric_matrix(vec3 v, float e = 1.0f)
{
    mat3 mat = mat3();
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            mat[i][j] = v[i] * v[j];
    return mat / e;
}

// Tear consecutively a vertex
bool tear_vertex(int vertex, cloth_structure &cloth, int remove = -1)
{
    std::vector<int> neighbors;
    if (cloth.should_break(vertex, neighbors, remove))
    {
        std::vector<vertex_infos> &vertices = cloth.vertices;
        numarray<vec3> &position = cloth.position;
        numarray<vec3> &normal = cloth.normal;

        size_t const N = vertices.size();

        vertex_infos nv = {};
        nv.force = vertices[vertex].force;
        nv.velocity = vertices[vertex].velocity;
        vertices.push_back(nv);
        position.push_back(vec3(position[vertex]));
        normal.push_back(vec3(normal[vertex]));

        std::vector<spring> ss = vertices[vertex].springs;
        vertices[vertex].springs = std::vector<spring>();

        for (spring s : ss)
        {
            if (std::find(neighbors.begin(), neighbors.end(), s.id) != neighbors.end())
            {
                vertices[N].springs.push_back(spring(s));
                for (spring &s_ : vertices[s.id].springs)
                    if (s_.id == vertex)
                        s_.id = N;
            }
            else
                vertices[vertex].springs.push_back(spring(s));
        }

        cloth.update_triangles(vertex, N, vertices[N].springs);
        return true;
    }
    return false;
}

// Fill the neighbors representing one of the sides of the tear
// Returns which way it represents (0 or 1 if no problem, 2 otherwise)
inline int find_neighbors(std::vector<vertex_infos> const& vertices, std::vector<int> &empty_sides,
    int border, int i, spring &left, spring &right, std::vector<int> &neighbors)
{
    int p[2] = { -1, -1 };
    int t = 0;
    for (spring left_spring : vertices[left.id].springs)
    {
        for (spring sub_spring : vertices[left_spring.id].springs)
            if (sub_spring.id == i)
            {
                p[t++] = left_spring.id;
                if (t == 2)
                    break;
            }
        if (t == 2)
            break;
    }

    t = 0;
    neighbors.push_back(left.id);
    neighbors.push_back(p[t]);
    while (1)
    {
        int current = neighbors[neighbors.size() - 1];
        if (std::find(empty_sides.begin(), empty_sides.end(), current) != empty_sides.end())
            break;
        
        for (spring sub_spring : vertices[current].springs)
        {
            if (std::find(neighbors.begin(), neighbors.end(), sub_spring.id) == neighbors.end())
            {
                if (sub_spring.id == right.id)
                {
                    neighbors.push_back(right.id);
                    break;
                }
                for (spring sub_sub_spring : vertices[sub_spring.id].springs)
                {
                    if (sub_sub_spring.id == i)
                    {
                        neighbors.push_back(sub_spring.id);
                        break;
                    }
                }
                if (neighbors[neighbors.size() - 1] != current)
                    break;
            }
        }

        if (neighbors[neighbors.size() - 1] == right.id)
            break;
        if (neighbors[neighbors.size() - 1] == current)
        {
            if (border)
                break;
            ++t;
            neighbors.clear();
            if (t == 2)
                break;
            // Couldn't link left to right with one of the side (there is probably a tear in the way), try with the other side
            neighbors.clear();
            neighbors.push_back(left.id);
            neighbors.push_back(p[t]);
        }
    }

    return t;
}

// Compute tearing on all vertices
size_t simulation_tearing(cloth_structure &cloth, simulation_parameters const &parameters, constraint_structure const &constraint)
{
    std::vector<int> garbage;
    std::vector<vertex_infos> &vertices = cloth.vertices;
    numarray<vec3> &position = cloth.position;
    numarray<vec3> &normal = cloth.normal;

    size_t N = vertices.size();
    float const K = parameters.K;

    size_t teared = 0;
    for (size_t i = 0; i < N; ++i)
    {
        // Can't tear apart a constraint
        if (constraint.fixed_sample.find(i) != constraint.fixed_sample.end())
            continue;

        vertex_infos &vertex = vertices[i];
        if (vertex.springs.size() < 4)
            continue;

        // Compute the tension on a vertex
        mat3 tension_matrix = mat3();
        vec3 total_strain = vec3();
        for (spring spr : vertex.springs)
        {
            vec3 vect = position[spr.id] - position[i];
            if (norm(vect) == 0)
                continue;
            vec3 strain = K * (norm(vect) - spr.rest) * normalize(vect);
            total_strain += strain;
            tension_matrix += get_symetric_matrix(strain);
        }
        tension_matrix -= get_symetric_matrix(total_strain);

        vec3 max_tension = tension_matrix[0];
        if (norm(tension_matrix[1]) > norm(max_tension))
            max_tension = tension_matrix[1];
        if (norm(tension_matrix[2]) > norm(max_tension))
            max_tension = tension_matrix[2];
        
        // `max_tension` is the maximal tension vector on the vertex
        // If the vertex can handle more resistance, skip
        if (norm(max_tension) < parameters.resistance)
            continue;

        /** Detect wheter or not we are on a border
         *   \   /
         *    \ /
         * --- A ---
         *    / \
         *   / X \
         * If X is a missing triangle (in the list of rendered triangles), then the vertex A is a border
         */
        std::vector<int> empty_sides; // List of springs missing a triangle
        int border = cloth.count_empty_side(i, empty_sides);

        vec3 right_vect = cross(max_tension, normal[i]);

        // Contains the springs we'll tear the vertex on
        spring left = {-1, 0};
        spring right = {-1, 0};

        float max_dot_left = -1;
        float max_dot_right = -1;
        for (spring spr : vertex.springs)
        {
            vec3 vect = position[spr.id] - position[i];
            float dot_right = dot(vect, right_vect);
            if (dot_right > max_dot_right)
            {
                max_dot_right = dot_right;
                right = spr;
            }
            if (-dot_right > max_dot_left)
            {
                max_dot_left = -dot_right;
                left = spr;
            }
        }

        if (border)
        {
            // Safe: can't duplicate a border spring
            if (cloth.howMuchTriangles(i, right.id) == 1)
            {
                // Safe: can't duplicate a border spring
                if (cloth.howMuchTriangles(i, left.id) == 1)
                    continue;
                right = left;
            }
            else if (max_dot_right > max_dot_left)
                left = right;
            // Safe: can't duplicate a border spring
            else if (cloth.howMuchTriangles(i, left.id) == 1)
                left = right;
            else
                right = left;
        }
        // Safe: could'nt find valid springs separate on ?
        else if (left.id == -1 || right.id == -1 || left.id == right.id)
            continue;

        std::vector<spring> springs = vertex.springs;

        // find the neighbors of one of the vertex that will be created after tearing (graph walk from `left` to `right`)
        std::vector<int> neighbors;
        int t = find_neighbors(cloth.vertices, empty_sides, border, i, left, right, neighbors);

        if (t == 2)
        {
            std::cout << "ERROR: finished possibilities without finding any." << std::endl;
            continue;
        }

        // Create our new vertex
        vertex.springs = std::vector<spring>();
        size_t new_id = vertices.size();
        vertex_infos new_vertex = { vertex.force, vertex.velocity, {} };
        vertices.push_back(new_vertex);
        position.push_back(vec3(position[i]));
        normal.push_back(vec3(normal[i]));

        // Link neighbors to each vertex
        for (spring spr : springs)
        {
            if (spr.id != left.id && spr.id != right.id)
            {
                if (std::find(neighbors.begin(), neighbors.end(), spr.id) == neighbors.end())
                {
                    vertices[new_id].springs.push_back(spr);
                    for (spring &s_ : vertices[spr.id].springs)
                        if (s_.id == i)
                            s_.id = new_id;
                }
                else
                    vertex.springs.push_back(spr);
            }
        }

        // Update OpenGL's buffer of vertices and triangles
        if (vertices[new_id].springs.size() == 0)
            cloth.update_triangle(i, new_id, left.id, right.id);
        cloth.update_triangles(i, new_id, vertices[new_id].springs);

        // Add to each vertex the spring the original vertex was teared on
        vertices[new_id].springs.push_back(spring(left));
        vertex.springs.push_back(spring(left));
        spring new_s = spring(left);
        new_s.id = new_id;
        vertices[left.id].springs.push_back(new_s);

        // We only use one break spring for a border, no need for right
        if (!border)
        {
            // Add to each vertex the second spring the original vertex was teared on
            vertices[new_id].springs.push_back(spring(right));
            vertex.springs.push_back(spring(right));
            new_s = spring(right);
            new_s.id = new_id;
            vertices[right.id].springs.push_back(new_s);
        }
        ++teared;

        // A constraint can't break
        if (constraint.fixed_sample.find(left.id) == constraint.fixed_sample.end())
        {
            if (tear_vertex(left.id, cloth, cloth.count_empty_side(right.id, garbage) >= 4 ? right.id : -1))
                ++teared;
        }

        // A constraint can't break
        if (constraint.fixed_sample.find(right.id) == constraint.fixed_sample.end())
        {
            if (!border && tear_vertex(right.id, cloth, cloth.count_empty_side(left.id, garbage) >= 4 ? left.id : -1))
                ++teared;
        }
    }

    return teared;
}

bool simulation_detect_divergence(cloth_structure const &cloth)
{
    bool simulation_diverged = false;
    const size_t N = cloth.vertices.size();
    for (size_t k = 0; simulation_diverged == false && k < N; ++k)
    {
        const float f = norm(cloth.vertices[k].force);
        const vec3 &p = cloth.position.at_unsafe(k);

        if (std::isnan(f)) // detect NaN in force
        {
            std::cout << "\n **** NaN detected in forces" << std::endl;
            simulation_diverged = true;
        }

        if (f > 600.0f) // detect strong force magnitude
        {
            std::cout << "\n **** Warning : Strong force magnitude detected " << f << " at vertex " << k << " ****" << std::endl;
            simulation_diverged = true;
        }

        if (std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z)) // detect NaN in position
        {
            std::cout << "\n **** NaN detected in positions" << std::endl;
            simulation_diverged = true;
        }
    }

    return simulation_diverged;
}
