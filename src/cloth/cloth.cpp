#include "cloth.hpp"
#include <algorithm>
#include <queue>

using namespace cgp;

void print_array(numarray<uint3> array, int width)
{

    for (int i = 0; i < array.size(); i++)
    {
        printf(" | (%d, %d, %d) |", array[i][0], array[i][1], array[i][2]);
        if (i % width == 0)
            printf("\n");
    }
    printf("\n");
}

int find_neighbor(const std::vector<vertex_infos> vertices, const int vertex, const int neighbor)
{
    std::vector<spring> springs = vertices[vertex].springs;
    for (unsigned int i = 0; i < springs.size(); i++)
    {
        if (springs[i].id == neighbor)
            return i;
    }

    return -1;
}

void add_neighbor_vertex(std::vector<vertex_infos> *vertices, int vertex_a, int vertex_b, float len, bool is_diag)
{

    if (vertex_a >= vertices->size() || vertex_b >= vertices->size() || vertex_a < 0 || vertex_b < 0)
        return;

    if (is_diag)
    {
        spring ab = {vertex_b, len};
        spring ba = {vertex_a, len};
        (*vertices)[vertex_a].springs.push_back(ab);
        (*vertices)[vertex_b].springs.push_back(ba);
    }
    else
    {
        // Avoid adding multiple times the same neighbor
        if (find_neighbor(*vertices, vertex_a, vertex_b) == -1)
        {
            spring ab = {vertex_b, len};
            spring ba = {vertex_a, len};
            (*vertices)[vertex_a].springs.push_back(ab);
            (*vertices)[vertex_b].springs.push_back(ba);
        }
    }
}

mesh generate_cloth(unsigned int height, unsigned int width, float len, vec3 start_pos, std::vector<vertex_infos> *vertices_infos, bool save_infos = false)
{
    vec3 actual_pos = start_pos;
    mesh shape;

    float len_border =  len / (static_cast<float>(width) - 1.0f);
    printf("Create new grid - H: %d, W: %d, Len border: %f\n", height, width, len_border);

    // Init all the vertices
    for (unsigned int i = 0; i < height; i++)
    {
        actual_pos = start_pos;
        actual_pos[1] += len_border * i;
        for (unsigned int j = 0; j < width; j++)
        {

            float const u = static_cast<float>(i) / (static_cast<float>(height) - 1.0f);
            float const v = static_cast<float>(j) / (static_cast<float>(width) - 1.0f);

            if (save_infos)
            {
                vertex_infos vertex;
                vertex.force = vec3(0, 0, 0);
                vertex.velocity = vec3(0, 0, 0);
                vertex.springs = std::vector<spring>();
                vertices_infos->push_back(vertex);
            }

            shape.position.push_back(actual_pos);
            shape.normal.push_back(vec3(0, 0, 1));
            shape.uv.push_back({u, v});

            actual_pos[0] += len_border;
        }

        actual_pos = start_pos;
        actual_pos[1] += len_border * (static_cast<float>(i) + 0.5f);
        actual_pos[0] += len_border / 2.0f;

        if (i < height - 1)
        {
            // Create intermediate vertex
            for (unsigned int j = 0; j < width - 1; j++)
            {

                float const u = (static_cast<float>(i) + 0.5f) / (static_cast<float>(height) - 1.0f);
                float const v = (static_cast<float>(j) + 0.5f) / (static_cast<float>(width) - 1.0f);

                if (save_infos)
                {
                    vertex_infos vertex;
                    vertex.springs = std::vector<spring>();
                    vertices_infos->push_back(vertex);
                }

                shape.position.push_back(actual_pos);
                shape.normal.push_back(vec3(0, 0, 1));
                shape.uv.push_back({u, v});

                actual_pos[0] += len_border;
            }
        }
    }

    // Triangle connectivity
    numarray<uint3> connectivity;
    int actual_index = 0;
    for (unsigned int i = 0; i < height - 1; i++)
    {
        actual_index += width;
        for (unsigned int j = 0; j < width - 1; j++)
        {

            unsigned int top_right = actual_index - width + 1;
            unsigned int top_left = actual_index - width;
            unsigned int bot_left = actual_index + width - 1;
            unsigned int bot_right = actual_index + width;

            connectivity.push_back({actual_index, top_left, top_right});
            connectivity.push_back({actual_index, top_right, bot_right});
            connectivity.push_back({actual_index, bot_right, bot_left});
            connectivity.push_back({actual_index, bot_left, top_left});

            if (save_infos)
            {

                float len_diag = norm(shape.position[top_left] - shape.position[actual_index]);

                add_neighbor_vertex(vertices_infos, actual_index, top_left, len_diag, true);
                add_neighbor_vertex(vertices_infos, actual_index, top_right, len_diag, true);
                add_neighbor_vertex(vertices_infos, actual_index, bot_left, len_diag, true);
                add_neighbor_vertex(vertices_infos, actual_index, bot_right, len_diag, true);

                add_neighbor_vertex(vertices_infos, top_left, top_right, len_border, false);
                add_neighbor_vertex(vertices_infos, top_left, bot_left, len_border, false);
                add_neighbor_vertex(vertices_infos, top_right, bot_right, len_border, false);
                add_neighbor_vertex(vertices_infos, bot_left, bot_right, len_border, false);
            }

            actual_index++;
        }
    }
    
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

    position.resize(N_samples_edge_arg * N_samples_edge_arg);
    normal.resize(N_samples_edge_arg * N_samples_edge_arg);
    vertices = std::vector<vertex_infos>();
    mesh const cloth_mesh = generate_cloth(N_samples_edge_arg, N_samples_edge_arg, len_border_cloth, {-(len_border_cloth / 2.0f), -(len_border_cloth / 2.0f), start_height_cloth}, &vertices, true);

    position = cloth_mesh.position;
    normal = cloth_mesh.normal;
    triangle_connectivity = cloth_mesh.connectivity;
    shape = cloth_mesh;
}

void cloth_structure::update_normal()
{
    normal_per_vertex(position, triangle_connectivity, normal, true);
}

int cloth_structure::N_samples_edge() const
{
    return position.size();
}

std::vector<int> findAllTriangles(cgp::numarray<cgp::uint3> triangles, unsigned int a, unsigned int b)
{
    std::vector<int> res = {-1, -1};
    int found = 0;
    for (unsigned int i = 0; i < triangles.size(); i++)
    {
        if (triangles[i][0] == a || triangles[i][1] == a || triangles[i][2] == a)
        {
            if (triangles[i][0] == b || triangles[i][1] == b || triangles[i][2] == b)
            {
                res[found] = i;
                found++;
                if (found == 2)
                    return res;
            }
        }
    }
    return res;
}

int cloth_structure::howMuchTriangles(unsigned int a, unsigned int b)
{
    int found = 0;
    for (unsigned int i = 0; i < triangle_connectivity.size(); i++)
        if (triangle_connectivity[i][0] == a || triangle_connectivity[i][1] == a || triangle_connectivity[i][2] == a)
            if (triangle_connectivity[i][0] == b || triangle_connectivity[i][1] == b || triangle_connectivity[i][2] == b)
                ++found;
    return found;
}

void cloth_structure::update_triangle(unsigned int a, unsigned int newA, unsigned int b, unsigned int c) 
{
    for (unsigned int i = 0; i < triangle_connectivity.size(); i++)
        if (triangle_connectivity[i][0] == a || triangle_connectivity[i][1] == a || triangle_connectivity[i][2] == a)
            if (triangle_connectivity[i][0] == b || triangle_connectivity[i][1] == b || triangle_connectivity[i][2] == b)
                if (triangle_connectivity[i][0] == c || triangle_connectivity[i][1] == c || triangle_connectivity[i][2] == c)
                {
                    triangle_connectivity[i] = { newA, b, c };
                    return;
                }
}

void cloth_structure::update_triangles(unsigned int oldVerticeIndex, unsigned int newVerticeIndex, std::vector<spring> springsChanged)
{
    for (unsigned int i = 0; i < springsChanged.size(); i++)
    {
        int neighboor = springsChanged[i].id;

        std::vector<int> found = findAllTriangles(triangle_connectivity, oldVerticeIndex, neighboor);
        if (found[0] == -1 && found[1] == -1)
            continue;

        for (int j = 0; j < 2; j++)
        {
            if (found[j] == -1)
                break;
            int index_found = found[j];
            if (triangle_connectivity[index_found][0] == oldVerticeIndex)
                triangle_connectivity[index_found][0] = newVerticeIndex;
            else if (triangle_connectivity[index_found][1] == oldVerticeIndex)
                triangle_connectivity[index_found][1] = newVerticeIndex;
            else
                triangle_connectivity[index_found][2] = newVerticeIndex;
        }
    }
    shape.uv.push_back(shape.uv[oldVerticeIndex]);
    shape.color.push_back(shape.color[oldVerticeIndex]);
}

int cloth_structure::count_empty_side(int vertex, std::vector<int> &values)
{
    int side_no_triangle = 0;
    values = std::vector<int>();
    std::vector<spring> springs = vertices[vertex].springs;

    for (int i = 0; i < springs.size(); i++)
    {
        int actNeighboor = springs[i].id;
        int found = howMuchTriangles(vertex, actNeighboor);
        if (found == 0)
        {
            printf("ERROR: No Triangle found using %d, %d vertices !\n", vertex, actNeighboor);
            continue;
        }
        else if (found == 1)
        {
            values.push_back(actNeighboor);
            side_no_triangle++;
        }
    }
    return side_no_triangle;
}

bool cloth_structure::should_break(int vertex, std::vector<int> &neighbors, int remove)
{
    std::vector<int> garbage;
    if (count_empty_side(vertex, garbage) < 4)
        return false;

    std::vector<spring> vertex_springs = vertices[vertex].springs;

    std::queue<int> neighbors_queue = std::queue<int>();
    neighbors_queue.push(vertex_springs[0].id);
    while (neighbors_queue.size() > 0)
    {
        int current_vertex = neighbors_queue.front();
        neighbors_queue.pop();

        if (current_vertex != remove && std::find(neighbors.begin(), neighbors.end(), current_vertex) == neighbors.end())
        {
            neighbors.push_back(current_vertex);
            std::vector<spring> current_springs = vertices[current_vertex].springs;
            for (spring s : current_springs)
                for (spring s2 : vertex_springs)
                    if (s.id == s2.id)
                    {
                        neighbors_queue.push(s.id);
                        break;
                    }
        }
    }

    return true;
}

void cloth_structure_drawable::initialize(int N_samples_edge, float len_border_cloth, float start_height_cloth)
{
    std::vector<vertex_infos> vert_inf = std::vector<vertex_infos>();
    mesh const cloth_mesh = generate_cloth(N_samples_edge, N_samples_edge, len_border_cloth, {-(len_border_cloth / 2.0f), -(len_border_cloth / 2.0f), start_height_cloth}, &vert_inf, false);
    drawable.clear();
    drawable.initialize_data_on_gpu(cloth_mesh);
    drawable.material.phong.specular = 0.0f;
    opengl_check;
}

void cloth_structure_drawable::update(cloth_structure &cloth, cgp::opengl_texture_image_structure cloth_texture)
{
    cloth.shape.position = cloth.position;
    cloth.shape.normal = cloth.normal;
    cloth.shape.connectivity = cloth.triangle_connectivity;

    drawable.clear();
    drawable.initialize_data_on_gpu(cloth.shape);
    drawable.material.phong.specular = 0.0f;
    drawable.texture = cloth_texture;
}

void draw(cloth_structure_drawable const &cloth_drawable, environment_generic_structure const &environment)
{
    draw(cloth_drawable.drawable, environment);
}
void draw_wireframe(cloth_structure_drawable const &cloth_drawable, environment_generic_structure const &environment)
{
    draw_wireframe(cloth_drawable.drawable, environment);
}
