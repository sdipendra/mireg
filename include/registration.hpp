#include <mireg.hpp>

bool multires_registration(std::vector<point>& reading, std::vector<point>& reference, std::vector<std::vector<long double>>& transformation_mat, float min_cell_size, int hist_size, std::string& map);

