#include <project_classes.hpp>

void point::get_point(std::ifstream& file)
{
	file>>coordinate(0)>>coordinate(1)>>coordinate(2)>>normal(0)>>normal(1)>>normal(2)>>ref>>gray;
}
