#include "simulation.hpp"

using namespace cgp;




// Fill value of force applied on each particle
// - Gravity
// - Drag
// - Spring force
// - Wind force
void simulation_compute_force(cloth_structure& cloth, simulation_parameters const& parameters)
{
    // direct access to the variables
    /*grid_2D<vec3>& force = cloth.force;
    grid_2D<vec3> const& position = cloth.position;
    grid_2D<vec3> const& velocity = cloth.velocity;
     grid_2D<vec3> const& normal = cloth.normal;

    size_t const N = cloth.position.size();       // total number of vertices
    size_t const N_edge = cloth.N_samples_edge(); // number of vertices in one dimension of the grid

    // Retrive simulation parameter
    float const K = parameters.K;              // spring stifness
    float const m = parameters.mass_total / N; // mass of a particle
    float const mu = parameters.mu;            // damping coefficient
    float const	L0 = 1.0f / (N_edge - 1.0f);   // rest length between two direct neighboring particle


    // Gravity
    const vec3 g = { 0,0,-9.81f };
    for (int ku = 0; ku < N_edge; ++ku)
        for (int kv = 0; kv < N_edge; ++kv)
            force(ku, kv) = m * g;

    // Drag
    for (int ku = 0; ku < N_edge; ++ku)
        for (int kv = 0; kv < N_edge; ++kv)
            force(ku, kv) += -mu * m * velocity(ku, kv);


    // TO DO: Add spring forces ...
    for (int ku = 0; ku < N_edge; ++ku) {
        for (int kv = 0; kv < N_edge; ++kv) {

            // 1-length springs
            for (int i = -1; i < 2; i ++) {
                for (int j = -1; j < 2; j++) {
                    if (ku+i < 0 || kv+j < 0 || ku+i >= N_edge || kv+j >= N_edge || (i == 0 && j == 0))
                        continue;

                    float rest_length = L0;
                    if (i != 0 && j != 0)
                        rest_length = sqrt(L0 * L0 * 2);
                    // Compare the actual distance between points with the rest distance
                    float no = norm(position(ku+i,kv+j) - position(ku, kv)) - rest_length;
                    force(ku, kv) += K * no * normalize(position(ku+i,kv+j) - position(ku, kv));
                }
            }

            // 2-length springs
            int iCoords[4] = {2, -2, 0, 0};
            int jCoords[4] = {0,0, 2, -2};
            for (int i = 0; i < 4; i ++) {
                if (ku+iCoords[i] < 0 || kv+jCoords[i] < 0 || ku+iCoords[i] >= N_edge || kv+jCoords[i] >= N_edge)
                    continue;
                float no = norm(position(ku+iCoords[i],kv+jCoords[i]) - position(ku, kv)) - 2 * L0;
                force(ku, kv) += K * no * normalize(position(ku+iCoords[i],kv+jCoords[i]) - position(ku, kv));
            }

        }
    }


    // Force du vent perpendiculaire au tissu
    // Force du vent dépend de l'angle avec lequel ça arrive

    // Add wind force     
    for (int ku = 0; ku < N_edge; ++ku) {
        for (int kv = 0; kv < N_edge; ++kv) {
            vec3 wind_force = parameters.wind.magnitude * parameters.wind.direction;
            force(ku,kv) += dot(-wind_force, normal(ku,kv)) * normal(ku,kv);
        }
    }*/
}

void simulation_numerical_integration(cloth_structure& cloth, simulation_parameters const& parameters, float dt)
{
    /*int const N_edge = cloth.N_samples_edge();
    float const m = parameters.mass_total/ static_cast<float>(N_edge);

    for (int ku = 0; ku < N_edge; ++ku) {
        for (int kv = 0; kv < N_edge; ++kv) {
            vec3& v = cloth.velocity(ku, kv);
            vec3& p = cloth.position(ku, kv);
            vec3 const& f = cloth.force(ku, kv);

            // Standard semi-implicit numerical integration
            v = v + dt * f / m;
            p = p + dt * v;
        }
    }*/
    
}

void simulation_apply_constraints(cloth_structure& cloth, constraint_structure const& constraint)
{
    // Fixed positions of the cloth
    /*for (auto const& it : constraint.fixed_sample) {
        position_contraint c = it.second;
        cloth.position(c.ku, c.kv) = c.position; // set the position to the fixed one
    }

    // To do: apply external constraints
    // For all vertex:
    //   If vertex is below floor level ...
    //   If vertex is inside collision sphere ...

    // Collision with floor
    size_t const N_edge = cloth.N_samples_edge();
    grid_2D<vec3>& velocity = cloth.velocity;
    grid_2D<vec3>& position = cloth.position;
    float const floor_height = 0.02f;
    float offset_sphere_collision = 0.01f;

    for (int i = 0; i < N_edge; i++) {
        for (int j = 0; j < N_edge; j ++) {
            // Just collision particle with floor
            if (position(i,j).z <= constraint.ground_z + floor_height)
            {
                position(i,j) = {position(i,j).x, position(i,j).y, constraint.ground_z + floor_height};
            }

            vec3 dist_from_sphere_center = position(i,j) - constraint.sphere.center;
            float distance = abs(norm(dist_from_sphere_center));

            if (distance <= constraint.sphere.radius + offset_sphere_collision) {
                // Move object back along the vector sphere
                position(i, j) = constraint.sphere.center + normalize(dist_from_sphere_center) * (constraint.sphere.radius + offset_sphere_collision);
                vec3 normal_contact = normalize(position(i,j) - constraint.sphere.center);
                velocity(i, j) += dot(-velocity(i,j), normal_contact) * normal_contact;
            }
        }
    }*/
}



bool simulation_detect_divergence(cloth_structure const& cloth)
{
    /*bool simulation_diverged = false;
    const size_t N = cloth.position.size();
    for (size_t k = 0; simulation_diverged == false && k < N; ++k)
    {
        const float f = norm(cloth.force.data.at_unsafe(k));
        const vec3& p = cloth.position.data.at_unsafe(k);

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

    return simulation_diverged;*/
}

