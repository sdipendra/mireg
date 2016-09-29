#ifndef _PROJECT_CLASSES_H_	// to include the header only once
#define _PROJECT_CLASSES_H_

#include <mireg.hpp>

struct point
{
	Eigen::Vector3d coordinate;
	Eigen::Vector3d normal;
	float ref;
	float gray;
	void get_point(std::ifstream& file);
};

#endif
