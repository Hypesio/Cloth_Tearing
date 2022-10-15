#include "simulation.hpp"

#include <iostream>

using namespace cgp;

void simulation_compute_force(cloth_structure& cloth, simulation_parameters const& parameters)
{
    std::vector<vertex_infos>& vertices = cloth.vertices;
    numarray<vec3> const& position = cloth.position;
    numarray<vec3> const& normal = cloth.normal;

    size_t const N = vertices.size();

    float const K = parameters.K;
    float const m = parameters.mass_total / N;
    float const mu = parameters.mu;

    const vec3 g = { 0, 0, -9.81f };
    for (vertex_infos& v : vertices)
    {
        v.force = m * g - mu * m * v.velocity; // Gravity + Drag Force
    }

    for (int i = 0; i < N; ++i)
    {
        vertex_infos& v = vertices[i];
        for (spring s : v.springs)
        {
            vec3 vn = position[s.id] - position[i];
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

void simulation_numerical_integration(cloth_structure& cloth, simulation_parameters const& parameters, float dt)
{
    std::vector<vertex_infos>& vertices = cloth.vertices;
    numarray<vec3>& position = cloth.position;

    size_t const N = vertices.size();
    float const m = parameters.mass_total / static_cast<float>(N);

    //std::cout << "old position: " << position[1] << std::endl;
    for (int i = 0; i < N; ++i)
    {
        vertex_infos& v = vertices[i];
        v.velocity += (dt * v.force / m);
        position[i] += (dt * v.velocity);
    }
    //std::cout << "position: " << position[1] << std::endl;
    //std::cout << "velocity: " << vertices[1].velocity << std::endl;
}

void simulation_apply_constraints(cloth_structure& cloth, constraint_structure const& constraint)
{
    std::vector<vertex_infos>& vertices = cloth.vertices;
    numarray<vec3>& position = cloth.position;

    size_t const N = vertices.size();

    float const floor_height = .001;
    float const sphere_collision = .01;

    for (auto const& it : constraint.fixed_sample)
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
            vertex_infos& v = vertices[i];
            vec3 normal = normalize(sp);
            
            position[i] = constraint.sphere.center + normalize(sp) * (constraint.sphere.radius + sphere_collision);
            v.velocity -= dot(v.velocity, normal) * normal;
        }
    }
}

mat3 get_symetric_matrix(vec3 v, float e = 1.0f) {
    mat3 mat;  
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j ++) {
            mat[i][j] = v[i] * v[j];
        }
    }
    return mat * e;
}

size_t simulation_tearing(cloth_structure& cloth, simulation_parameters const& parameters)
{
    std::vector<vertex_infos>& vertices = cloth.vertices;
    numarray<vec3>& position = cloth.position;
    numarray<vec3>& normal = cloth.normal;
    
    size_t const N = vertices.size();

    size_t teared = 0;

    for (size_t k = 0; k < N; ++k)
    {
        vertex_infos& v = vertices[k];








        
        if (v.springs.size() > 3 && norm(v.force) > parameters.resistance)
        {
            vec3 orth1 = cross(v.force, normal[k]);
            vec3 orth2 = -orth1;

            spring s1 = v.springs[0];
            spring s2 = s1;
            for (spring s : v.springs)
            {
                vec3 tmp = position[s.id] - position[k];
                float d = dot(tmp, orth1);
                if (d > 0)
                {
                    if (d < dot(position[s1.id] - position[k], orth1))
                        s1 = s;
                }
                else
                {
                    if (dot(tmp, orth2) < dot(position[s2.id] - position[k], orth2))
                        s2 = s;
                }
            }

            size_t new_id = vertices.size();
            vertex_infos new_v = {};
            new_v.force = v.force;
            new_v.velocity = v.velocity;
            vertices.push_back(new_v);
            position.push_back(vec3(position[k]));
            normal.push_back(vec3(normal[k]));

            std::vector<spring> springs = v.springs;
            v.springs = std::vector<spring>();

            for (spring s : springs)
            {
                if (s.id != s1.id && s.id != s2.id)
                {
                    vec3 tmp = position[s.id] - position[k];
                    float d = dot(tmp, v.force);
                    if (d > 0)
                        vertices[new_id].springs.push_back(spring(s));
                    else
                        v.springs.push_back(spring(s));
                }
            }

            //if (vertices[new_id].springs.size() > )
            cloth.update_triangles(k, new_id, vertices[new_id].springs);

            vertices[new_id].springs.push_back(spring(s1));
            v.springs.push_back(spring(s1));
            vertices[new_id].springs.push_back(spring(s2));
            v.springs.push_back(spring(s2));
            ++teared;
        }
    }
    
    return teared;
}

bool simulation_detect_divergence(cloth_structure const& cloth)
{
    bool simulation_diverged = false;
    const size_t N = cloth.vertices.size();
    for (size_t k = 0; simulation_diverged == false && k < N; ++k)
    {
        const float f = norm(cloth.vertices[k].force);
        const vec3& p = cloth.position.at_unsafe(k);

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

