#pragma once

struct gui_parameters {
	bool display_frame = true;
	bool display_wireframe = false;
	int N_sample_edge = 10; 
	float lengh_cloth = 1.0f; 
	float height_cloth = 0.7f;
	vec3 pointAPosition; 
	vec3 pointBPosition; 
	int scene_type = 0;
};