#include <mireg.hpp>
#include <project_classes.hpp>

void plot(std::vector<std::vector<float>>& cloud);

void plot_ground(std::vector<std::vector<float>>& ground, std::vector<std::vector<float>>& rest);

void plot_plane(std::vector<std::vector<float>>& ground, std::vector<std::vector<float>>& rest, std::vector<float>& normal, std::vector<float>& centroid);

void plot_scans(std::vector<std::vector<float>>& reading, std::vector<std::vector<float>>& reference);

void plot_merged(std::vector<point>& reading, std::vector<point>& reference);

void plot_feature_map(std::pair<std::pair<int, int>, std::vector<std::vector<float>>>& map);

void plot_feature_map_merged(std::pair<std::pair<int, int>, std::vector<std::vector<float>>>& map1, std::pair<std::pair<int, int>, std::vector<std::vector<float>>>& map2);

