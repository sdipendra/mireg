#include <mireg.hpp>
#include <project_classes.hpp>

void plot(std::vector<std::vector<double>>& cloud);

void plot_ground(std::vector<std::vector<double>>& ground, std::vector<std::vector<double>>& rest);

void plot_plane(std::vector<std::vector<double>>& ground, std::vector<std::vector<double>>& rest, std::vector<double>& normal, std::vector<double>& centroid);

void plot_scans(std::vector<std::vector<double>>& reading, std::vector<std::vector<double>>& reference);

void plot_merged(std::vector<point>& reading, std::vector<point>& reference);

void plot_feature_map(std::pair<std::pair<int, int>, std::vector<std::vector<double>>>& map);

void plot_feature_map_merged(std::pair<std::pair<int, int>, std::vector<std::vector<double>>>& map1, std::pair<std::pair<int, int>, std::vector<std::vector<double>>>& map2);

