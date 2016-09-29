#ifndef _PROJECT_CLASSES_H_	// to include the header only once
#define _PROJECT_CLASSES_H_

#include <mireg.hpp>

struct point
{
	float x, y, z;
	float nx, ny, nz;
	float ref;
	float gray;
	void get_point(std::ifstream& file);
};

#endif
