#include "constraint.hpp"

using namespace cgp;

void constraint_structure::set_fixed_point_scene(cloth_structure const &cloth, int scene_type, int n_sample, gui_parameters &gui)
{
    // Reset fixed point
    fixed_sample.clear();
	setA.clear();
	setB.clear();

    gui.pointAPosition = cloth.position[0];
    gui.pointBPosition = cloth.position[n_sample - 1];

    // Two top corner hold the tissue
    if (scene_type == 0)
    {
        setA[0] = vec3(0, 0, 0);
        setB[n_sample - 1] = vec3(0, 0, 0);
        add_fixed_position(0, cloth);
        add_fixed_position(n_sample - 1, cloth);
    }
    // Both side hold the tissue
    else if (scene_type == 1 || scene_type == 2)
    {
        unsigned int i = 0;
        while (i < cloth.vertices.size())
        {
            add_fixed_position(i, cloth);
            setA[i] = cloth.position[i] - gui.pointAPosition;
            i += n_sample - 1;
            add_fixed_position(i, cloth);
            setB[i] = cloth.position[i] - gui.pointBPosition;
            i += n_sample;
        }
    }
}

void constraint_structure::add_fixed_position(int i, cloth_structure const &cloth)
{
    fixed_sample[i] = {i, cloth.position[i]};
}

void constraint_structure::update_position(int i, vec3 position)
{
    fixed_sample[i].position = position;
}

void constraint_structure::update_positions(int set_index, vec3 position)
{

    std::map<size_t, cgp::vec3> set = setA;
    if (set_index != 0)
        set = setB;
    for (auto const &it : set)
    {
        cgp::vec3 offset = it.second;
        update_position(it.first, position + offset);
    }
}

void constraint_structure::remove_fixed_position(int i)
{
    fixed_sample.erase(i);
}