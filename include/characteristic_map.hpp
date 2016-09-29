#include <mireg.hpp>
#include <project_classes.hpp>

void variance_map(std::vector<point>& cloud, float cell_size, std::pair<pii, vvf>& map);

void normal_map(std::vector<point>& cloud, float cell_size, std::pair<pii, vvf>& map);

void reflectivity_map(std::vector<point>& cloud, float cell_size, std::pair<pii, vvf>& map);

void grayscale_map(std::vector<point>& cloud, float cell_size, std::pair<pii, vvf>& map);

