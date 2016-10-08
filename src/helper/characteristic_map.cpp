#include <project_classes.hpp>
#include <characteristic_map.hpp>
#include <transformation.hpp>

void variance_map(std::vector<point>& cloud, double cell_size, std::vector<std::pair<pii, double>>& feature_map)
{
	std::vector<point> ground, rest;
	ground_plane_extraction(cloud, ground, rest);	//TODO: Give proof of correctness that ground points give irrelevant information, if possible extend it to reflectivity_map, grayscale_map and normal_map as it decreases runtime by half as the data points are reduced by approximately half.

	int size = rest.size();
	std::vector<std::pair<pii, double>> variances(size);
	for(int i=0; i<size; ++i)
	{
		int x=(rest[i].coordinate(0)+0.5*cell_size)/cell_size, y=(rest[i].coordinate(1)+0.5*cell_size)/cell_size;
		double value = rest[i].coordinate(2) * rest[i].coordinate(2);
		variances[i] = mp(mp(x, y), value);
	}
	
	std::sort(variances.begin(), variances.end());	// TODO: Current bottle-neck
	variances.push_back(mp(mp(std::numeric_limits<int>::max(), std::numeric_limits<int>::max()), std::numeric_limits<double>::max()));
	int variances_size = variances.size();
	pii curr = variances[0].first;
	double mean = variances[0].second;
	int count=1;
	feature_map.clear(); feature_map.reserve(variances_size);
	for(int i=1; i<variances_size; ++i)
	{
		if(variances[i].first != curr)
		{
			mean /= count;
			feature_map.push_back(mp(curr, mean));
			
			curr=variances[i].first;
			mean=variances[i].second;
			count=1;
		}
		else
		{
			mean += variances[i].second;
			count++;
		}
	}
}

void reflectivity_map(std::vector<point>& cloud, double cell_size, std::vector<std::pair<pii, double>>& feature_map)
{
	int size = cloud.size();
	std::vector<std::pair<pii, double>> reflectivities(size);
	for(int i=0; i<size; ++i)
	{
		int x=(cloud[i].coordinate(0)+0.5*cell_size)/cell_size, y=(cloud[i].coordinate(1)+0.5*cell_size)/cell_size;
		double value = cloud[i].ref * cloud[i].ref;
		reflectivities[i] = mp(mp(x, y), value);
	}

	std::sort(reflectivities.begin(), reflectivities.end());	// TODO: Current bottle-neck
	reflectivities.push_back(mp(mp(std::numeric_limits<int>::max(), std::numeric_limits<int>::max()), std::numeric_limits<double>::max()));
	int reflectivities_size = reflectivities.size();
	pii curr = reflectivities[0].first;
	double mean = reflectivities[0].second;
	int count=1;
	feature_map.clear(); feature_map.reserve(reflectivities_size);
	for(int i=1; i<reflectivities_size; ++i)
	{
		if(reflectivities[i].first != curr)
		{
			mean /= count;
			feature_map.push_back(mp(curr, mean));
			
			curr=reflectivities[i].first;
			mean=reflectivities[i].second;
			count=1;
		}
		else
		{
			mean += reflectivities[i].second;
			count++;
		}
	}
}

void grayscale_map(std::vector<point>& cloud, double cell_size, std::vector<std::pair<pii, double>>& feature_map)
{
	int size = cloud.size();
	std::vector<std::pair<pii, double>> grayscales(size);
	for(int i=0; i<size; ++i)
	{
		int x=(cloud[i].coordinate(0)+0.5*cell_size)/cell_size, y=(cloud[i].coordinate(1)+0.5*cell_size)/cell_size;
		double value = cloud[i].gray * cloud[i].gray;
		grayscales[i] = mp(mp(x, y), value);
	}

	std::sort(grayscales.begin(), grayscales.end());	// TODO: Current bottle-neck
	grayscales.push_back(mp(mp(std::numeric_limits<int>::max(), std::numeric_limits<int>::max()), std::numeric_limits<double>::max()));
	int grayscales_size = grayscales.size();
	pii curr = grayscales[0].first;
	double mean = grayscales[0].second;
	int count=1;
	feature_map.clear(); feature_map.reserve(grayscales_size);
	for(int i=1; i<grayscales_size; ++i)
	{
		if(grayscales[i].first != curr)
		{
			mean /= count;
			feature_map.push_back(mp(curr, mean));
			
			curr=grayscales[i].first;
			mean=grayscales[i].second;
			count=1;
		}
		else
		{
			mean += grayscales[i].second;
			count++;
		}
	}

}

void normal_map(std::vector<point>& cloud, double cell_size, std::vector<std::pair<pii, double>>& feature_map)
{
	int size = cloud.size();
	std::vector<std::pair<pii, Eigen::Vector3d>> normals; normals.reserve(size);
	for(int i=0; i<size; ++i)
	{
		int x=(cloud[i].coordinate(0)+0.5*cell_size)/cell_size, y=(cloud[i].coordinate(1)+0.5*cell_size)/cell_size;
		Eigen::Vector3d value = cloud[i].normal;
		if(value!=Eigen::Vector3d::Zero())
		{
			value.normalize();
			normals.push_back(mp(mp(x, y), value));
		}
	}
	
	struct normals_comp
	{
		bool operator()(const std::pair<pii, Eigen::Vector3d>& lhs, const std::pair<pii, Eigen::Vector3d>& rhs)
		{
			if(lhs.first < rhs.first) return true;
			else return false;
		}
	} my_comp;

	std::sort(normals.begin(), normals.end(), my_comp);	// TODO: Current bottle-neck
	normals.push_back(mp(mp(std::numeric_limits<int>::max(), std::numeric_limits<int>::max()), Eigen::Vector3d::Zero()));
	int normals_size = normals.size();
	pii curr = normals[0].first;
	Eigen::Vector3d mean = normals[0].second;
	int count=1, prev=0;
	feature_map.clear(); feature_map.reserve(normals_size);
	for(int i=1; i<normals_size; ++i)
	{
		if(normals[i].first != curr)
		{
			mean /= count;
			mean.normalize();
			Eigen::Matrix3d mat = Eigen::Matrix3d::Zero();
			for(int j=prev; j<i; ++j)
			{
				mat += (normals[j].second - mean)*(normals[j].second - mean).transpose();
			}
			mat /= count;
			double data[] = {mat(0, 0), mat(0, 1), mat(0, 2),
			mat(1, 0), mat(1, 1), mat(1, 2),
			mat(2, 0), mat(2, 1), mat(2, 2)};
			gsl_matrix_view m = gsl_matrix_view_array(data, 3, 3);
			gsl_vector *eval = gsl_vector_alloc(3);
			gsl_matrix *evec = gsl_matrix_alloc(3, 3);
			gsl_eigen_symmv_workspace * w = gsl_eigen_symmv_alloc(3);
			gsl_eigen_symmv(&m.matrix, eval, evec, w);
			gsl_eigen_symmv_free(w);
			gsl_eigen_symmv_sort(eval, evec, GSL_EIGEN_SORT_ABS_DESC);
			feature_map.push_back(mp(curr, std::abs(gsl_vector_get(eval, 0))));
			
			gsl_vector_free (eval);
			gsl_matrix_free (evec);
			curr=normals[i].first;
			mean=normals[i].second;
			count=1;
			prev=i;
		}
		else
		{
			mean += normals[i].second;
			count++;
		}
	}
}
