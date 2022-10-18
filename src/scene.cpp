#include "scene.hpp"

using namespace cgp;

void scene_structure::initialize()
{
	camera_control.initialize(inputs, window); // Give access to the inputs and window global state to the camera controler
	camera_control.set_rotation_axis_z();
	camera_control.look_at({ 3.0f, 2.0f, 2.0f }, {0,0,0}, {0,0,1});
	global_frame.initialize_data_on_gpu(mesh_primitive_frame());

    obstacle_floor.initialize_data_on_gpu(mesh_primitive_quadrangle({ -5,-5,0 }, { -5,5,0 }, { 5,5,0 }, { 5,-5,0 }));
    obstacle_floor.texture.load_and_initialize_texture_2d_on_gpu("assets/tiles.png");
	obstacle_floor.model.translation = { 0,0,constraint.ground_z };
	obstacle_floor.material.texture_settings.two_sided = true;

	obstacle_sphere.initialize_data_on_gpu(mesh_primitive_sphere());
	obstacle_sphere.model.translation = constraint.sphere.center;
	obstacle_sphere.model.scaling = constraint.sphere.radius;
	obstacle_sphere.material.color = { 1,0,0 };

	sphere_fixed_position.initialize_data_on_gpu(mesh_primitive_sphere());
	sphere_fixed_position.model.scaling = 0.02f;
	sphere_fixed_position.material.color = { 0,0,1 };

	cloth_texture.load_and_initialize_texture_2d_on_gpu("assets/cloth.jpg");

	initialize_cloth(gui.N_sample_edge, gui.lengh_cloth, gui.height_cloth);
}

// Compute a new cloth in its initial position (can be called multiple times)
void scene_structure::initialize_cloth(int N_sample, float len_border_cloth, float start_height_cloth)
{
	cloth.initialize(N_sample, len_border_cloth, start_height_cloth);
	cloth_drawable.initialize(N_sample, len_border_cloth, start_height_cloth);
	cloth_drawable.drawable.texture = cloth_texture;
	cloth_drawable.drawable.material.texture_settings.two_sided = true;
	constraint.sphere.center = {0.0f, 0.0f, 0.0f};
	obstacle_sphere.model.translation = {0.0f, 0.0f, 0.0f};
	//obstacle_sphere.hierarchy_transform_model.translation = vec3(0, 0, 0);
	constraint.set_fixed_point_scene(cloth, gui.scene_type, N_sample, gui);
}


void scene_structure::display_frame()
{
	// Set the light to the current position of the camera
	environment.light = camera_control.camera_model.position();
	environment.background_color = {.55, .75, .95};
	
	if (gui.display_frame)
		draw(global_frame, environment);


	// Elements of the scene: Obstacles (floor, sphere), and fixed position
	// ***************************************** //
	// Scene 2 move the sphere
	if (gui.scene_type == 2) 
	{
		constraint.sphere.center += vec3(0, 0, 0.0032);
		obstacle_sphere.model.translation += vec3(0, 0, 0.0032);
	}
	constraint.sphere.radius = gui.sphere_radius; 
	obstacle_sphere.model.scaling = gui.sphere_radius;
	draw(obstacle_floor, environment);
	draw(obstacle_sphere, environment);
	

	for (auto const& c : constraint.fixed_sample)
	{
		sphere_fixed_position.model.translation = c.second.position;
		draw(sphere_fixed_position, environment);
	}

	// Update constraint positions
	constraint.update_positions(0, gui.pointAPosition);
	constraint.update_positions(1, gui.pointBPosition);
	
	// Simulation of the cloth
	// ***************************************** //
	int const N_step = 10; // Adapt here the number of intermediate simulation steps (ex. 5 intermediate steps per frame)
	for (int k_step = 0; simulation_running == true && k_step < N_step; ++k_step)
	{
		// Update the forces on each particle
		simulation_compute_force(cloth, parameters);

		// One step of numerical integration
		simulation_numerical_integration(cloth, parameters, parameters.dt / N_step);

		// Apply the positional (and velocity) constraints
		simulation_apply_constraints(cloth, constraint);

		// Tear the cloth on vertices with too much tension
		simulation_tearing(cloth, parameters, constraint);

		// Check if the simulation has not diverged - otherwise stop it
		bool const simulation_diverged = simulation_detect_divergence(cloth);
		if (simulation_diverged) {
			std::cout << "\n *** Simulation has diverged ***" << std::endl;
			std::cout << " > The simulation is stoped" << std::endl;
			simulation_running = false;
		}
	}


	// Cloth display
	// ***************************************** //

	// Prepare to display the updated cloth
	cloth.update_normal();        // compute the new normals
	cloth_drawable.update(cloth, cloth_texture); // update the positions on the GPU

	// Display the cloth
	draw(cloth_drawable, environment);
	if (gui.display_wireframe)
		draw_wireframe(cloth_drawable, environment);
		
	
}

void scene_structure::display_gui()
{
	bool reset = false;

	ImGui::Text("Display");
	reset |= ImGui::SliderInt("Scene", &gui.scene_type, 0, 2);
	ImGui::Checkbox("Frame", &gui.display_frame);
	ImGui::Checkbox("Wireframe", &gui.display_wireframe);
	ImGui::Checkbox("Texture Cloth", &cloth_drawable.drawable.material.texture_settings.active);

	ImGui::Spacing(); ImGui::Spacing();

	ImGui::Text("Simulation parameters");
	ImGui::SliderFloat("Time step", &parameters.dt, 0.01f, 0.2f, "%.3f", 0.01f);
	ImGui::SliderFloat("Stiffness", &parameters.K, 1.0f, 80.0f, "%.3f", 2.0f);
	ImGui::SliderFloat("Wind magnitude", &parameters.wind.magnitude, 0, 1, "%.3f", 2.0f);
	ImGui::SliderFloat("Damping", &parameters.mu, 1.0f, 100.0f);
	ImGui::SliderFloat("Mass", &parameters.mass_total, 0.2f, 5.0f, "%.3f", 2.0f);
	ImGui::SliderFloat("Freedom fighter", &parameters.resistance, 0.2f, 40.0f, "%.3f", 1.0f);

	ImGui::Spacing(); ImGui::Spacing();

	ImGui::SliderFloat("Point A: x", &gui.pointAPosition[0], -1.00f, 1.0f, "%.3f", 1.0f);
	ImGui::SliderFloat("Point A: y", &gui.pointAPosition[1], -1.00f, 1.0f, "%.3f", 1.0f);
	ImGui::SliderFloat("Point A: z", &gui.pointAPosition[2], -1.00f, 1.0f, "%.3f", 1.0f);

	ImGui::SliderFloat("Point B: x", &gui.pointBPosition[0], -1.0f, 1.0f, "%.3f", 2.0f);
	ImGui::SliderFloat("Point B: y", &gui.pointBPosition[1], -1.0f, 1.0f, "%.3f", 2.0f);
	ImGui::SliderFloat("Point B: z", &gui.pointBPosition[2], -1.0f, 1.0f, "%.3f", 2.0f);
	ImGui::SliderFloat("Sphere Radius: ", &gui.sphere_radius, 0.0, 1.0f, "%.3f", 2.0f);

	ImGui::Spacing(); ImGui::Spacing();

	reset |= ImGui::SliderInt("Cloth samples", &gui.N_sample_edge, 4, 32);
	reset |= ImGui::SliderFloat("Cloth length", &gui.lengh_cloth, 0.2f, 4.0f, "%.3f", 1.0f);
	reset |= ImGui::SliderFloat("Cloth start height", &gui.height_cloth, 0.2f, 4.0f, "%.3f", 0.7f);

	ImGui::Spacing(); ImGui::Spacing();
	reset |= ImGui::Button("Restart");
	if (reset) {
		initialize_cloth(gui.N_sample_edge, gui.lengh_cloth, gui.height_cloth);
		simulation_running = true;
	}
}

void scene_structure::mouse_move_event()
{
	if (!inputs.keyboard.shift)
		camera_control.action_mouse_move(environment.camera_view);
}
void scene_structure::mouse_click_event()
{
	camera_control.action_mouse_click(environment.camera_view);
}
void scene_structure::keyboard_event()
{
	camera_control.action_keyboard(environment.camera_view);
}
void scene_structure::idle_frame()
{
	camera_control.idle_frame(environment.camera_view);
}

