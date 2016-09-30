#include <mireg.hpp>
#include <project_classes.hpp>

void variance_map(std::vector<point>& cloud, double cell_size, std::pair<pii, vvd>& map);

void normal_map(std::vector<point>& cloud, double cell_size, std::pair<pii, vvd>& map);

void reflectivity_map(std::vector<point>& cloud, double cell_size, std::pair<pii, vvd>& map);

void grayscale_map(std::vector<point>& cloud, double cell_size, std::pair<pii, vvd>& map);

