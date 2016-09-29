#include <mireg.hpp>
#include <project_classes.hpp>

void ground_plane_extraction(std::vector<point>& complete, std::vector<point>& ground, std::vector<point>& rest);

void transform(std::vector<point>& cloud, Eigen::Matrix4d& transformation_mat);

void build_transform_normal(Eigen::Matrix4d& transformation_mat, Eigen::Vector3d& normal1, Eigen::Vector3d& normal2);

void build_transform_centroid(Eigen::Matrix4d& transformation_mat, Eigen::Vector3d& normal1, Eigen::Vector3d& centroid1, Eigen::Vector3d& normal2, Eigen::Vector3d& centroid2);

void build_transform(Eigen::Matrix4d& transformation_mat, long double theta, long double x, long double y);

std::vector<std::vector<long double>> mat_multi(std::vector<std::vector<long double>>& a, std::vector<std::vector<long double>>& b);

std::vector<std::vector<long double>> mat_add(std::vector<std::vector<long double>>& a, std::vector<std::vector<long double>>& b);

Eigen::Matrix4d mat_inv(Eigen::Matrix4d& mat);

std::vector<long double> euler_rep(Eigen::Matrix4d& transformation_mat);


//void transform(std::vector<point>& cloud, std::vector<long double>& normal, std::vector<long double>& centroid);

//void transform(std::vector<point>& cloud, std::vector<std::vector<long double>>& transformation_mat, long double theta, long double x, long double y);

//std::vector<std::vector<long double>> mat_multi(std::vector<std::vector<long double>>& a, std::vector<std::vector<long double>>& b);

//std::vector<std::vector<long double>> mat_add(std::vector<std::vector<long double>>& a, std::vector<std::vector<long double>>& b);

//void transform(std::vector<std::vector<long double>>& transformation_mat, std::vector<long double>& normal, std::vector<long double>& centroid);

//void transform_normal(std::vector<point>& cloud, std::vector<long double>& normal);

