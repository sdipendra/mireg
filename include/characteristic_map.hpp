#include <mireg.hpp>
#include <project_classes.hpp>

void variance_map(std::vector<point>& cloud, double cell_size, std::vector<std::pair<pii, double>>& feature_map);

void normal_map(std::vector<point>& cloud, double cell_size, std::vector<std::pair<pii, double>>& feature_map);

void reflectivity_map(std::vector<point>& cloud, double cell_size, std::vector<std::pair<pii, double>>& feature_map);

void grayscale_map(std::vector<point>& cloud, double cell_size, std::vector<std::pair<pii, double>>& feature_map);

