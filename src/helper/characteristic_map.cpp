#include <characteristic_map.h>
#include <transformation.h>

void variance_map(std::vector<point>& cloud, float cell_size, std::pair<pii, vvf>& map)
{
	float maxx=-std::numeric_limits<float>::max(), maxy=-std::numeric_limits<float>::max(), maxz=-std::numeric_limits<float>::max();
	float minx=std::numeric_limits<float>::max(), miny=std::numeric_limits<float>::max(), minz=std::numeric_limits<float>::max();
	for(auto it : cloud)
	{
		if(it.x>maxx) maxx=it.x;
		if(it.y>maxy) maxy=it.y;
		if(it.z>maxz) maxz=it.z;
		if(it.x<minx) minx=it.x;
		if(it.y<miny) miny=it.y;
		if(it.z<minz) minz=it.z;
	}
	std::vector<point> ground, rest;
	ground_plane_extraction(cloud, ground, rest);
	
	int rows=(1.5*cell_size+maxx)/cell_size+(0.5*cell_size-minx)/cell_size, cols=(1.5*cell_size+maxy)/cell_size+(0.5*cell_size-miny)/cell_size;
	map.first.first=(0.5*cell_size-minx)/cell_size, map.first.second=(0.5*cell_size-miny)/cell_size;
	
	map.second.clear();
	map.second=vvf(rows, vf(cols, 0.0));
	vvi point_count(rows, vi(cols, 0));

	for(auto it : rest)
	{
		int x=(it.x-minx+0.5*cell_size)/cell_size, y=(it.y-miny+0.5*cell_size)/cell_size;
		map.second[x][y]+=it.z*it.z;
		point_count[x][y]++;
	}

	for(int i=0; i<rows; ++i) for(int j=0; j<cols; ++j) if(point_count[i][j]!=0) map.second[i][j]/=float(point_count[i][j]);
}

void reflectivity_map(std::vector<point>& cloud, float cell_size, std::pair<pii, vvf>& map)
{
	float maxx=-std::numeric_limits<float>::max(), maxy=-std::numeric_limits<float>::max(), maxz=-std::numeric_limits<float>::max();
	float minx=std::numeric_limits<float>::max(), miny=std::numeric_limits<float>::max(), minz=std::numeric_limits<float>::max();
	for(auto it : cloud)
	{
		if(it.x>maxx) maxx=it.x;
		if(it.y>maxy) maxy=it.y;
		if(it.z>maxz) maxz=it.z;
		if(it.x<minx) minx=it.x;
		if(it.y<miny) miny=it.y;
		if(it.z<minz) minz=it.z;
	}
	
	int rows=(1.5*cell_size+maxx)/cell_size+(0.5*cell_size-minx)/cell_size, cols=(1.5*cell_size+maxy)/cell_size+(0.5*cell_size-miny)/cell_size;
	map.first.first=(0.5*cell_size-minx)/cell_size, map.first.second=(0.5*cell_size-miny)/cell_size;
	
	map.second.clear();
	map.second=vvf(rows, vf(cols, 0.0));
	vvi point_count(rows, vi(cols, 0));

	for(auto it : cloud)
	{
		int x=(it.x-minx+0.5*cell_size)/cell_size, y=(it.y-miny+0.5*cell_size)/cell_size;
		map.second[x][y]+=it.ref*it.ref;
		point_count[x][y]++;
	}

	for(int i=0; i<rows; ++i) for(int j=0; j<cols; ++j) if(point_count[i][j]!=0) map.second[i][j]/=float(point_count[i][j]);
}

void grayscale_map(std::vector<point>& cloud, float cell_size, std::pair<pii, vvf>& map)
{
	float maxx=-std::numeric_limits<float>::max(), maxy=-std::numeric_limits<float>::max(), maxz=-std::numeric_limits<float>::max();
	float minx=std::numeric_limits<float>::max(), miny=std::numeric_limits<float>::max(), minz=std::numeric_limits<float>::max();
	for(auto it : cloud)
	{
		if(it.x>maxx) maxx=it.x;
		if(it.y>maxy) maxy=it.y;
		if(it.z>maxz) maxz=it.z;
		if(it.x<minx) minx=it.x;
		if(it.y<miny) miny=it.y;
		if(it.z<minz) minz=it.z;
	}
	
	int rows=(1.5*cell_size+maxx)/cell_size+(0.5*cell_size-minx)/cell_size, cols=(1.5*cell_size+maxy)/cell_size+(0.5*cell_size-miny)/cell_size;
	map.first.first=(0.5*cell_size-minx)/cell_size, map.first.second=(0.5*cell_size-miny)/cell_size;
	
	map.second.clear();
	map.second=vvf(rows, vf(cols, 0.0));
	vvi point_count(rows, vi(cols, 0));

	for(auto it : cloud)
	{
		int x=(it.x-minx+0.5*cell_size)/cell_size, y=(it.y-miny+0.5*cell_size)/cell_size;
		map.second[x][y]+=it.gray*it.gray;
		point_count[x][y]++;
	}

	for(int i=0; i<rows; ++i) for(int j=0; j<cols; ++j) if(point_count[i][j]!=0) map.second[i][j]/=float(point_count[i][j]);
}

void normal_map(std::vector<point>& cloud, float cell_size, std::pair<pii, vvf>& map)
{
	float maxx=-std::numeric_limits<float>::max(), maxy=-std::numeric_limits<float>::max(), maxz=-std::numeric_limits<float>::max();
	float minx=std::numeric_limits<float>::max(), miny=std::numeric_limits<float>::max(), minz=std::numeric_limits<float>::max();
	for(auto it : cloud)
	{
		if(it.x>maxx) maxx=it.x;
		if(it.y>maxy) maxy=it.y;
		if(it.z>maxz) maxz=it.z;
		if(it.x<minx) minx=it.x;
		if(it.y<miny) miny=it.y;
		if(it.z<minz) minz=it.z;
	}
	
	int rows=(1.5*cell_size+maxx)/cell_size+(0.5*cell_size-minx)/cell_size, cols=(1.5*cell_size+maxy)/cell_size+(0.5*cell_size-miny)/cell_size;
	map.first.first=(0.5*cell_size-minx)/cell_size, map.first.second=(0.5*cell_size-miny)/cell_size;
	
	struct norm
	{
		float nx, ny, nz;
		norm()
		{
			nx=0.0; ny=0.0; nz=0.0;
		}
	};
	std::vector<std::vector<norm>> mean_norms(rows, std::vector<norm>(cols, norm()));
	vvi points_count(rows, vi(cols, 0));
	for(auto it : cloud)
	{
		int x=(it.x-minx+0.5*cell_size)/cell_size, y=(it.y-miny+0.5*cell_size)/cell_size;
		mean_norms[x][y].nx+=it.nx;
		mean_norms[x][y].ny+=it.ny;
		mean_norms[x][y].nz+=it.nz;
		++points_count[x][y];
	}
	for(int i=0; i<rows; ++i)
	{
		for(int j=0; j<cols; ++j)
		{
			if(points_count[i][j]!=0)
			{
				mean_norms[i][j].nx/=float(points_count[i][j]);
				mean_norms[i][j].ny/=float(points_count[i][j]);
				mean_norms[i][j].nz/=float(points_count[i][j]);
			}
		}
	}
	struct mat
	{
		float a1, a2, a3;
		float b1, b2, b3;
		float c1, c2, c3;
		mat()
		{
			a1=0.0; a2=0.0; a3=0.0; b1=0.0; b2=0.0; b3=0.0; c1=0.0; c2=0.0; c3=0.0;
		}
	};
	
	std::vector<std::vector<mat>> matrix(rows, std::vector<mat>(cols, mat()));
	for(auto it : cloud)
	{
		int x=(it.x-minx+0.5*cell_size)/cell_size, y=(it.y-miny+0.5*cell_size)/cell_size;
		float px=it.nx-mean_norms[x][y].nx, py=it.ny-mean_norms[x][y].ny, pz=it.nz-mean_norms[x][y].nz;
		matrix[x][y].a1+= px*px; matrix[x][y].a2+= px*py; matrix[x][y].a3+= px*pz;
		matrix[x][y].b1+= py*px; matrix[x][y].b2+= py*py; matrix[x][y].b3+= py*pz;
		matrix[x][y].c1+= pz*px; matrix[x][y].c2+= pz*py; matrix[x][y].c3+= pz*pz;
	}

	map.second.clear();
	map.second=vvf(rows, vf(cols, 0.0));
	for(int i=0; i<rows; ++i)
	{
		for(int j=0; j<cols; ++j)
		{
			if(points_count[i][j]!=0)
			{
				matrix[i][j].a1/=float(points_count[i][j]); matrix[i][j].a2/=float(points_count[i][j]); matrix[i][j].a3/=float(points_count[i][j]);
				matrix[i][j].b1/=float(points_count[i][j]); matrix[i][j].b2/=float(points_count[i][j]); matrix[i][j].b3/=float(points_count[i][j]);
				matrix[i][j].c1/=float(points_count[i][j]); matrix[i][j].c2/=float(points_count[i][j]); matrix[i][j].c3/=float(points_count[i][j]);
				
				double data[] = {matrix[i][j].a1, matrix[i][j].a2, matrix[i][j].a3,
				matrix[i][j].b1, matrix[i][j].b2, matrix[i][j].b3,
				matrix[i][j].c1, matrix[i][j].c2, matrix[i][j].c3};

				gsl_matrix_view m = gsl_matrix_view_array(data, 3, 3);
				gsl_vector *eval = gsl_vector_alloc(3);
				gsl_matrix *evec = gsl_matrix_alloc(3, 3);

				gsl_eigen_symmv_workspace * w = gsl_eigen_symmv_alloc(3);
				gsl_eigen_symmv(&m.matrix, eval, evec, w);
				gsl_eigen_symmv_free(w);
				gsl_eigen_symmv_sort(eval, evec, GSL_EIGEN_SORT_ABS_DESC);
				map.second[i][j]=std::abs(gsl_vector_get(eval, 0));
				
				gsl_vector_free (eval);
				gsl_matrix_free (evec);
			}
		}
	}
}
