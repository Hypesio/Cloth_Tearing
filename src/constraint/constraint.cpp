#include "constraint.hpp"

using namespace cgp;

void constraint_structure::add_fixed_position(int i, cloth_structure const& cloth)
{
	fixed_sample[i] = { i, cloth.position[i] };
}

void constraint_structure::update_positions(int i, vec3 position)
{
	fixed_sample[i].position = position;
}

void constraint_structure::remove_fixed_position(int i)
{
	fixed_sample.erase(i);
}