#include <mireg.hpp>
#include <project_classes.hpp>

void best_fit_plane(std::vector<point>& ground, std::vector<long double>& normal, std::vector<long double>& centroid);

void best_fit_plane1(std::vector<point>& ground, Eigen::Vector3d& normal, Eigen::Vector3d& centroid);

