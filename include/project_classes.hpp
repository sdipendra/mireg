#ifndef _PROJECT_CLASSES_H_	// to include the header only once
#define _PROJECT_CLASSES_H_

#include <mireg.hpp>

struct point
{
	Eigen::Vector3d coordinate;	// pack the data more effeciently
	Eigen::Vector3d normal;
	float ref;
	float gray;
	void get_point(std::ifstream& file);
};

struct fmap
{
	pii bottom_left, top_right;
	std::vector<std::pair<pii, double>> values;
};

struct information_node
{
	float x, y, v;
};


#endif
