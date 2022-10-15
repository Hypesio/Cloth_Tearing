#include "simulation.hpp"

#include <iostream>
#include <algorithm>

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
    mat3 mat = mat3();  
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            mat[i][j] = v[i] * v[j];
        }
    };
    return mat / e;
}

size_t simulation_tearing(cloth_structure& cloth, simulation_parameters const& parameters)
{

    for (vertex_infos vertex : cloth.vertices) 
    {
        for (spring s : vertex.springs) 
        {
            if (s.id >= cloth.vertices.size()) {
                printf("\n\n **** Error springs out of bounds ! Before*** \n\n");
            }
        }
    }

    std::vector<vertex_infos>& vertices = cloth.vertices;
    numarray<vec3>& position = cloth.position;
    numarray<vec3>& normal = cloth.normal;
    
    size_t const N = vertices.size();
    float const K = parameters.K;

    size_t teared = 0;

    for (size_t k = 0; k < N; ++k)
    {
        vertex_infos& v = vertices[k];

        if (v.springs.size() < 3)
            continue;

        mat3 c = mat3();
        vec3 strain_total = vec3();
        for (spring s : v.springs)
        {
            vec3 vn = position[s.id] - position[k];
            if (norm(vn) == 0)
                continue;
            vec3 strain = K * (norm(vn) - s.rest) * normalize(vn);
            strain_total += strain;
            c += get_symetric_matrix(strain);
        }
        
        c -= get_symetric_matrix(strain_total);

        vec3 tension = c[0];
        if (norm(c[1]) > norm(tension))
            tension = c[1];
        if (norm(c[2]) > norm(tension))
            tension = c[2];
        
        if (norm(tension) > parameters.resistance)
        {
            vec3 orth1 = cross(tension, normal[k]);
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
            vertex_infos new_v = { v.force, v.velocity, {} };
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
                    float d = dot(tmp, tension);
                    if (d > 0)
                    {
                        vertices[new_id].springs.push_back(s);
                        for (spring& s_ : vertices[s.id].springs)
                            if (s_.id == k)
                                s_.id = new_id;
                    }
                    else
                        v.springs.push_back(s);
                }
            }

            cloth.update_triangles(k, new_id, vertices[new_id].springs);

            vertices[new_id].springs.push_back(spring(s1));
            v.springs.push_back(spring(s1));
            s1.id = new_id;
            vertices[s1.id].springs.push_back(s1);

            vertices[new_id].springs.push_back(spring(s2));
            v.springs.push_back(spring(s2));
            s2.id = new_id;
            vertices[s2.id].springs.push_back(s2);

            ++teared;

            std::vector<int> n1;
            if (false && cloth.should_break(s1.id, n1))
            {
                size_t nid = vertices.size();
                vertex_infos nv = {};
                nv.force = vertices[s1.id].force;
                nv.velocity = vertices[s1.id].velocity;
                vertices.push_back(nv);
                position.push_back(vec3(position[s1.id]));
                normal.push_back(vec3(normal[s1.id]));

                std::vector<spring> ss = vertices[s1.id].springs;
                vertices[s1.id].springs = std::vector<spring>();

                for (spring s : ss)
                {
                    if (std::find(n1.begin(), n1.end(), s.id) != n1.end())
                    {
                        vertices[nid].springs.push_back(spring(s));
                        for (spring& s_ : vertices[s.id].springs)
                            if (s_.id == s1.id)
                                s_.id = nid;
                    }
                    else
                        vertices[s1.id].springs.push_back(spring(s));
                }

                cloth.update_triangles(s1.id, nid, vertices[nid].springs);
                ++teared;
            }

            std::vector<int> n2;
            if (false && cloth.should_break(s2.id, n2))
            {
                size_t nid = vertices.size();
                vertex_infos nv = {};
                nv.force = vertices[s2.id].force;
                nv.velocity = vertices[s2.id].velocity;
                vertices.push_back(nv);
                position.push_back(vec3(position[s2.id]));
                normal.push_back(vec3(normal[s2.id]));

                std::vector<spring> ss = vertices[s2.id].springs;
                vertices[s2.id].springs = std::vector<spring>();

                for (spring s : ss)
                {
                    if (std::find(n2.begin(), n2.end(), s.id) != n2.end())
                    {
                        vertices[nid].springs.push_back(spring(s));
                        for (spring& s_ : vertices[s.id].springs)
                            if (s_.id == s2.id)
                                s_.id = nid;
                    }
                    else
                        vertices[s2.id].springs.push_back(spring(s));
                }

                cloth.update_triangles(s2.id, nid, vertices[nid].springs);
                ++teared;
            }
        }
    }

    for (vertex_infos vertex : vertices) 
    {
        for (spring s : vertex.springs) 
        {
            if (s.id >= vertices.size()) {
                printf("\n\n **** Error springs out of bounds ! After*** \n\n");
            }
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

