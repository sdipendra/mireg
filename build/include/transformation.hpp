#include <mireg.hpp>
#include <project_classes.hpp>

void ground_plane_extraction(std::vector<point>& complete, std::vector<point>& ground, std::vector<point>& rest);

void transform(std::vector<point>& cloud, Eigen::Matrix4d& transformation_mat);

void build_transform_xy(Eigen::Matrix4d& transformation_mat, Eigen::Vector3d& normal1, Eigen::Vector3d& centroid1, Eigen::Vector3d& normal2, Eigen::Vector3d& centroid2);

void build_transform_z(Eigen::Matrix4d& transformation_mat, Eigen::Vector3d& euler);

std::vector<double> euler_rep(Eigen::Matrix4d& transformation_mat);
