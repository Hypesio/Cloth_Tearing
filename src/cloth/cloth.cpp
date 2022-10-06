#include "cloth.hpp"

using namespace cgp;

void print_array(numarray<uint3> array, int width) {

    for (int i = 0; i < array.size(); i++) {
        printf(" | (%d, %d, %d) |", array[i][0], array[i][1], array[i][2]); 
        if (i % width == 0)
            printf("\n");
    }
    printf("\n");
}

mesh generate_cloth(unsigned int height,unsigned int width, vec3 start_pos, float total_len_border) {
    vec3 actual_pos = start_pos; 
    mesh shape; 

    

    float len_border = total_len_border / (float)(width - 1);
    printf("Create new grid - H: %d, W: %d, Len border: %f\n", height, width, len_border);

    // Init all the vertices
    for (unsigned int i = 0; i < height; i ++) {
        actual_pos = start_pos; 
        actual_pos[1] += len_border * i;
        for (unsigned int j = 0; j < width; j ++) {
            
            float const u = i/(height-1.0f);
			float const v = j/(width-1.0f);

            shape.position.push_back(actual_pos); 
            shape.normal.push_back(vec3(0, 0, 1));
            shape.uv.push_back({u, v});

            actual_pos[0] += len_border; 
        }

        actual_pos = start_pos; 
        actual_pos[1] += len_border * ((float)i + 0.5f);
        actual_pos[0] += len_border / 2.0f; 

        if (i < height - 1) {
            // Create intermediate vertex 
            for (unsigned int j = 0; j < width - 1; j++) {
                
                float const u = (i + 0.5f) /(height-1.0f);
                float const v = (j + 0.5f) /(width-1.0f);

                shape.position.push_back(actual_pos); 
                shape.normal.push_back(vec3(0, 0, 1));
                shape.uv.push_back({u, v});

                actual_pos[0] += len_border; 
            }
        }
        
    }
    

    // Triangle connectivity 
    numarray<uint3> connectivity; 
    unsigned int actual_index = 0; 
    for (unsigned int i = 0; i < height - 1; i++) {
        actual_index += width; 
        for (unsigned int j = 0; j < width - 1; j++) {
            
            unsigned int top_right = actual_index - width + 1; 
            unsigned int top_left = actual_index - width; 
            unsigned int bot_left = actual_index + width - 1; 
            unsigned int bot_right = actual_index + width; 

            connectivity.push_back({actual_index, top_left, top_right}); 
            connectivity.push_back({actual_index, top_right, bot_right});
            connectivity.push_back({actual_index, bot_right, bot_left});
            connectivity.push_back({actual_index, bot_left, top_left});

            actual_index ++;
        }
    }

    //print_array(connectivity, width);
    //printf("%d\n", shape.position.size());
    shape.connectivity = connectivity; 
	shape.fill_empty_field();
	shape.flip_connectivity();

    return shape; 
}

void cloth_structure::initialize(int N_samples_edge_arg, float len_border_cloth, float start_height_cloth)
{
    assert_cgp(N_samples_edge_arg > 3, "N_samples_edge=" + str(N_samples_edge_arg) + " should be > 3");

    position.clear();
    normal.clear();
    vertices.clear();
    //velocity.clear();
    //force.clear();

    position.resize(N_samples_edge_arg * N_samples_edge_arg);
    normal.resize(N_samples_edge_arg * N_samples_edge_arg);
    //velocity.resize(N_samples_edge_arg, N_samples_edge_arg);
    //force.resize(N_samples_edge_arg, N_samples_edge_arg);

    mesh const cloth_mesh = generate_cloth(N_samples_edge_arg, N_samples_edge_arg, { -(len_border_cloth/2.0f),0,start_height_cloth }, len_border_cloth);

    position = cloth_mesh.position;
    normal = cloth_mesh.normal;
    triangle_connectivity = cloth_mesh.connectivity;
}

void cloth_structure::update_normal()
{
    normal_per_vertex(position, triangle_connectivity, normal);
}

int cloth_structure::N_samples_edge() const
{
    return position.size();
}



void cloth_structure_drawable::initialize(int N_samples_edge, float len_border_cloth, float start_height_cloth)
{
    mesh const cloth_mesh = generate_cloth(N_samples_edge, N_samples_edge, { -(len_border_cloth/2.0f),0, 1.0f }, start_height_cloth);//mesh_primitive_grid({ 0,0,0 }, {1,0,0 }, { 1,1,0 }, { 0,1,0 }, N_samples_edge, N_samples_edge);

    drawable.clear();
    drawable.initialize_data_on_gpu(cloth_mesh);
    drawable.material.phong.specular = 0.0f;
    opengl_check;
}


void cloth_structure_drawable::update(cloth_structure const& cloth)
{    
    drawable.vbo_position.update(cloth.position.data);
    drawable.vbo_normal.update(cloth.normal.data);
}

void draw(cloth_structure_drawable const& cloth_drawable, environment_generic_structure const& environment)
{
    draw(cloth_drawable.drawable, environment);
}
void draw_wireframe(cloth_structure_drawable const& cloth_drawable, environment_generic_structure const& environment)
{
    draw_wireframe(cloth_drawable.drawable, environment);
}