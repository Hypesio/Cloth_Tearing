#include "simulation.hpp"

#include <iostream>
#include <algorithm>

using namespace cgp;

void debug_print_springs(int vertex, std::vector<spring> springs)
{
    printf("Springs of %d: ", vertex);
    for (spring s : springs)
    {
        printf("%u ,", s.id);
    }
    printf("\n");
}

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

void simulation_numerical_integration(cloth_structure &cloth, simulation_parameters const &parameters, float dt)
{
    std::vector<vertex_infos> &vertices = cloth.vertices;
    numarray<vec3> &position = cloth.position;

    size_t const N = vertices.size();
    float const m = parameters.mass_total / static_cast<float>(N);

    // std::cout << "old position: " << position[1] << std::endl;
    for (int i = 0; i < N; ++i)
    {
        vertex_infos &v = vertices[i];
        v.velocity += (dt * v.force / m);
        position[i] += (dt * v.velocity);
    }
    // std::cout << "position: " << position[1] << std::endl;
    // std::cout << "velocity: " << vertices[1].velocity << std::endl;
}

void simulation_apply_constraints(cloth_structure &cloth, constraint_structure const &constraint)
{
    std::vector<vertex_infos> &vertices = cloth.vertices;
    numarray<vec3> &position = cloth.position;

    size_t const N = vertices.size();

    float const floor_height = .001;
    float const sphere_collision = .01;

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

mat3 get_symetric_matrix(vec3 v, float e = 1.0f)
{
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

bool tear_vertex(int vertex, cloth_structure &cloth, int remove = -1)
{
    std::vector<int> neighbors;
    if (cloth.should_break(vertex, neighbors, remove))
    {
        printf("Launch consecutive break on %d\n", vertex);
        for (int n : neighbors)
            std::cout << n << ';';
        std::cout << std::endl;
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
        debug_print_springs(vertex, vertices[vertex].springs);
        debug_print_springs(N, vertices[N].springs);
        return true;
    }
    return false;
}

size_t simulation_tearing(cloth_structure &cloth, simulation_parameters const &parameters, constraint_structure const &constraint)
{
    std::vector<vertex_infos> &vertices = cloth.vertices;
    numarray<vec3> &position = cloth.position;
    numarray<vec3> &normal = cloth.normal;

    size_t N = vertices.size();
    float const K = parameters.K;

    size_t teared = 0;
    for (size_t i = 0; i < N; ++i)
    {
        if (constraint.fixed_sample.find(i) != constraint.fixed_sample.end())
            continue;

        vertex_infos &vertex = vertices[i];
        if (vertex.springs.size() < 4)
            continue;

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
        
        if (norm(max_tension) > parameters.resistance)
        {
            std::cout << std::endl << std::endl << "SPLITING " << i << std::endl;

            int empty = cloth.count_empty_side(i);
            std::cout << "This vertex has " << empty << " one-sided springs." << std::endl;
            if (empty)
                std::cout << "`-> BORDER!" << std::endl;

            std::cout << "Vertex's springs before changes: (" << vertex.springs.size() << ')' << std::endl;
            debug_print_springs(i, vertex.springs);

            vec3 right_vect = cross(max_tension, normal[i]);

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

            if (empty)
            {
                if (cloth.howMuchTriangles(i, right.id) == 1)
                {
                    if (cloth.howMuchTriangles(i, left.id) == 1)
                        continue;
                    right = left;
                }
                else if (max_dot_right > max_dot_left)
                    left = right;
                else if (cloth.howMuchTriangles(i, left.id) == 1)
                    left = right;
                else
                    right = left;

                std::cout << "Choosen spring is " << left.id << std::endl;
            }
            else if (left.id == -1 || right.id == -1 || left.id == right.id)
            {
                std::cout << "ERROR: left.id: " << left.id << "; right.id: " << right.id << std::endl;
                continue;
            }
            else
            {
                std::cout << "Choosen springs are " << left.id << "; " << right.id << std::endl;
            }

            std::vector<spring> springs = vertex.springs;
            vertex.springs = std::vector<spring>();

            int p[2] = { -1, -1 };
            std::vector<int> neighbors;
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
                {
                    std::cout << "break right in neighbor" << std::endl;
                    break;
                }
                if (neighbors[neighbors.size() - 1] == current)
                {
                    if (empty)
                    {
                        std::cout << "break border" << std::endl;
                        break;
                    }
                    ++t;
                    if (t == 2)
                        break;
                    neighbors.clear();
                    neighbors.push_back(left.id);
                    neighbors.push_back(p[t]);
                }
            }

            for (int abdwo : neighbors)
                std::cout << abdwo << ';';
            std::cout << std::endl;

            if (t == 2)
            {
                std::cout << "ERROR: finished possibilities without finding any." << std::endl;
                continue;
            }

            size_t new_id = vertices.size();
            vertex_infos new_vertex = { vertex.force, vertex.velocity, {} };
            vertices.push_back(new_vertex);
            position.push_back(vec3(position[i]));
            normal.push_back(vec3(normal[i]));

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

            debug_print_springs(i, vertex.springs);
            debug_print_springs(new_id, vertices[new_id].springs);

            if (vertices[new_id].springs.size() == 0)
                cloth.update_triangle(i, new_id, left.id, right.id);
            else
                cloth.update_triangles(i, new_id, vertices[new_id].springs);

            vertices[new_id].springs.push_back(spring(left));
            vertex.springs.push_back(spring(left));
            spring new_s = spring(left);
            new_s.id = new_id;
            vertices[left.id].springs.push_back(new_s);

            // We only use one break spring for a border, no need for s2
            if (!empty)
            {
                vertices[new_id].springs.push_back(spring(right));
                vertex.springs.push_back(spring(right));
                new_s = spring(right);
                new_s.id = new_id;
                vertices[right.id].springs.push_back(new_s);
            }

            printf("New Size v%lu  -- Size new V %lu --- S1 %d S2 %d V %lu\n", vertex.springs.size(), vertices[new_id].springs.size(), left.id, right.id, i);
            debug_print_springs(i, vertex.springs);
            debug_print_springs(new_id, vertices[new_id].springs);
            debug_print_springs(left.id, vertices[left.id].springs);
            debug_print_springs(right.id, vertices[right.id].springs);
            ++teared;

            if (tear_vertex(left.id, cloth, cloth.count_empty_side(right.id) >= 4 ? right.id : -1))
                ++teared;

            if (!empty && tear_vertex(right.id, cloth, cloth.count_empty_side(left.id) >= 4 ? left.id : -1))
                ++teared;
        }
    }

    N = vertices.size();
    for (vertex_infos vertex : cloth.vertices)
        for (spring s : vertex.springs)
            if (s.id >= N)
                printf("\n\n **** Error springs out of bounds ! Before*** \n\n");

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
