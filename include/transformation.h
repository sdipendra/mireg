#include <mireg.h>

void ground_plane_extraction(std::vector<point>& complete, std::vector<point>& ground, std::vector<point>& rest);

void transform(std::vector<point>& cloud, std::vector<std::vector<long double>>& transformation_mat);

void build_transform_normal(std::vector<std::vector<long double>>& transformation_mat, std::vector<long double>& normal1, std::vector<long double>& normal2);

void build_transform_centroid(std::vector<std::vector<long double>>& transformation_mat, std::vector<long double>& normal1, std::vector<long double>& centroid1, std::vector<long double>& normal2, std::vector<long double>& centroid2);

void build_transform(std::vector<std::vector<long double>>& transformation_mat, long double theta, long double x, long double y);

std::vector<std::vector<long double>> mat_multi(std::vector<std::vector<long double>>& a, std::vector<std::vector<long double>>& b);

std::vector<std::vector<long double>> mat_add(std::vector<std::vector<long double>>& a, std::vector<std::vector<long double>>& b);

std::vector<std::vector<long double>> mat_inv(std::vector<std::vector<long double>>& mat);

std::vector<long double> euler_rep(std::vector<std::vector<long double>>& transformation_mat);


//void transform(std::vector<point>& cloud, std::vector<long double>& normal, std::vector<long double>& centroid);

//void transform(std::vector<point>& cloud, std::vector<std::vector<long double>>& transformation_mat, long double theta, long double x, long double y);

//std::vector<std::vector<long double>> mat_multi(std::vector<std::vector<long double>>& a, std::vector<std::vector<long double>>& b);

//std::vector<std::vector<long double>> mat_add(std::vector<std::vector<long double>>& a, std::vector<std::vector<long double>>& b);

//void transform(std::vector<std::vector<long double>>& transformation_mat, std::vector<long double>& normal, std::vector<long double>& centroid);

//void transform_normal(std::vector<point>& cloud, std::vector<long double>& normal);

