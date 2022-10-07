# Goal

Given a list of points, calculate all points' forces (and compute its destruction). PseudoC++Code

# Code

```cpp
void simulation_compute_force(cloth_structure& cloth, simulation_parameters const& parameters)
{
    vector<struct vertex>& vertices = cloth.vertices;
    vector<vec3> const& position = cloth.position;
    vector<vec3> const& normal = cloth.normal;

    size_t const N = vertices.size();

    float const K = parameters.K;
    float const m = parameters.mass_total / N;
    float const mu = parameters.mu;
    float const	L0 = 1.0f / (N_edge - 1.0f);
    vec3 wind = parameters.wind.magnitude * parameters.wind.direction;

    const vec3 g = { 0, 0, -9.81f };
    for (struct vertex v : vertices)
        v.force = m * g - mu * m * v.velocity; // Gravity + Drag Force

    for (int i = 0; i < N; ++i) {
        struct vertex v = vertices[i];
        for (struct spring s : v.neighbors) {
            vec3 vn = position[s.id] - position[i];
            v.force += K * (norm(vn) - s.rest - L0) * (vn) / norm(vn); // Spring Force
        }
    }

    for (int i = 0; i < N; ++i)
        vertices[i].force += dot(-wind, normal[i]) * normal[i]; // Wind Force
}
```

```cpp
void simulation_numerical_integration(cloth_structure& cloth, simulation_parameters const& parameters, float dt)
{
    vector<struct vertex>& vertices = cloth.vertices;
    vector<vec3>& position = cloth.position;

    size_t const N = vertices.size();
    float const m = parameters.mass_total / static_cast<float>(N);

    for (int i = 0; i < N; ++i) {
        struct vertex v = vertices[i];
        v.velocity += (dt * v.force / m);
        position[i] += (dt * v.velocity);
    } 
}
```

```cpp
void simulation_apply_constraints(cloth_structure& cloth, constraint_structure const& constraint)
{
    vector<struct vertex>& vertices = cloth.vertices;
    vector<vec3>& position = cloth.position;

    size_t const N = vertices.size();

    float const floor_height = 0.02f;
    float const sphere_collision = 0.01f;

    for (auto const& it : constraint.fixed_sample) {
        position_contraint c = it.second;
        position[c.i] = c.position;
    }

    for (int i = 0; i < N; i++) {
        if (position[i].z <= constraint.ground_z + floor_height)
        {
            position[i] = { position[i].x, position[i].y, constraint.ground_z + floor_height };
        }

        vec3 sp = position[i] - constraint.sphere.center;
        if (norm(sp) <= constraint.sphere.radius + sphere_collision)
        {
            struct vertex v = vertices[i];
            vec3 normal = normalize(sp);
            
            position[i] = constraint.sphere.center + normalize(sp) * (constraint.sphere.radius + sphere_collision);
            v.velocity += dot(-v.velocity, normal) * normal;
        }
    }
}
```