#include <project_classes.hpp>

void point::get_point(std::ifstream& file)
{
	file>>x>>y>>z>>nx>>ny>>nz>>ref>>gray;
}

