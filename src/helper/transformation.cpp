#include <project_classes.hpp>
#include <transformation.hpp>

const int grid_length=100, grid_breadth=100;
const float clearance=0.1;

std::vector<std::vector<long double>> mat_multi(std::vector<std::vector<long double>>& a, std::vector<std::vector<long double>>& b)	// assuming a[0].size()==b.size() and no 0x0 mat
{
	std::vector<std::vector<long double>> ans(a.size(), std::vector<long double>(b[0].size(), 0.0));
	for(int i=0; i<int(a.size()); ++i)
	{
		for(int j=0; j<int(b[0].size()); ++j)
		{
			for(int k=0; k<int(b.size()); ++k)
			{
				ans[i][j]+=a[i][k]*b[k][j];
			}
		}
	}
	return ans;
}

std::vector<std::vector<long double>> mat_add(std::vector<std::vector<long double>>& a, std::vector<std::vector<long double>>& b)	// assuming same dimensions
{
	std::vector<std::vector<long double>> ans(a.size(), std::vector<long double>(a[0].size(), 0.0));
	for(int i=0; i<int(a.size()); ++i)
	{
		for(int j=0; j<int(a[0].size()); ++j)
		{
			ans[i][j]=a[i][j]+b[i][j];
		}
	}
	return ans;
}

std::vector<std::vector<long double>> mat_inv(std::vector<std::vector<long double>>& mat)	// Assuming only 4x4 transformation matrix
{
	std::vector<std::vector<long double>> ans(mat.size(), std::vector<long double>(mat[0].size(), 0.0));
	for(int i=0; i<3; ++i)
	{
		for(int j=0; j<3; ++j)
		{
			ans[i][j]=mat[j][i];
		}
	}
	for(int i=0; i<3; ++i)
	{
		ans[i][3]=-(ans[i][0]*mat[0][3]+ans[i][1]*mat[1][3]+ans[i][2]*mat[2][3]);
	}
	ans[3][3]=1.0;
	return ans;
}

bool equal(const long double& n1, const long double& n2, long double epsilon=std::numeric_limits<long double>::epsilon())
{
	return(epsilon>std::abs(n1-n2));
}

std::vector<long double> euler_rep(std::vector<std::vector<long double>>& transformation_mat)
{
	std::vector<long double> ans(6);
	
	for(int i=0; i<3; ++i) ans[i]=transformation_mat[i][3];	// translations
	
	// check for gimbal locking
	if(equal(transformation_mat[2][0], (long double)-1.0))
	{
		ans[3] = pi/2.0;
		ans[4] = 0.0;
		ans[5] = ans[3]+atan2(transformation_mat[0][1], transformation_mat[0][2]);
	}
	else if(equal(transformation_mat[2][0], (long double)1.0))
	{
		ans[3] = -pi/2.0;
		ans[4] = 0.0;
		ans[5] = -ans[3] + atan2(-transformation_mat[0][1], -transformation_mat[0][2]);
	}
	else
	{
		long double y1 = -asin(transformation_mat[2][0]);
		long double y2 = pi - y1;
		
		long double x1 = atan2(transformation_mat[2][1] / cos(y1), transformation_mat[2][2] / cos(y1));
		long double x2 = atan2(transformation_mat[2][1] / cos(y2), transformation_mat[2][2] / cos(y2));
		
		long double z1 = atan2(transformation_mat[1][0] / cos(y1), transformation_mat[0][0] / cos(y1));
		long double z2 = atan2(transformation_mat[1][0] / cos(y2), transformation_mat[0][0] / cos(y2));
		
		if((std::abs(x1) + std::abs(y1) + std::abs(z1)) <= (std::abs(x2) + std::abs(y2) + std::abs(z2)))
		{
			ans[3]=x1;
			ans[4]=y1;
			ans[5]=z1;
		}
		else
		{
			ans[3]=x2;
			ans[4]=y2;
			ans[5]=z2;
		}
	}
	for(int i=3; i<6; ++i)
	{
		ans[i]*=(180/pi);
	}
	return ans;
}

void ground_plane_extraction(std::vector<point>& complete, std::vector<point>& ground, std::vector<point>& rest)
{
	float maxx=-std::numeric_limits<float>::max(), maxy=-std::numeric_limits<float>::max(), maxz=-std::numeric_limits<float>::max(), minx=std::numeric_limits<float>::max(), miny=std::numeric_limits<float>::max(), minz=std::numeric_limits<float>::max();
	for(auto it=complete.begin(); it!=complete.end(); ++it)
	{
		if(it->x>maxx) maxx=it->x;
		if(it->y>maxy) maxy=it->y;
		if(it->z>maxz) maxz=it->z;
		
		if(it->x<minx) minx=it->x;
		if(it->y<miny) miny=it->y;
		if(it->z<minz) minz=it->z;
	}
	
	float length=maxx-minx, breadth=maxy-miny, height=maxz-minz;
	float xbox=(length)/(grid_length-1), ybox=(breadth)/(grid_breadth-1);
	vvf grid(grid_length, vf(grid_breadth, std::numeric_limits<float>::max()));
	for(auto it=complete.begin(); it!=complete.end(); ++it)
	{
		int x=(it->x-minx+xbox/2.0)/xbox, y=(it->y-miny+ybox/2.0)/ybox;
		if(grid[x][y]>it->z) grid[x][y]=it->z;
	}
	
	ground.clear(); rest.clear();
	for(auto it=complete.begin(); it!=complete.end(); ++it)
	{
		int x=(it->x-minx+xbox/2.0)/xbox, y=(it->y-miny+ybox/2.0)/ybox;
		if(it->z-grid[x][y]<clearance) ground.push_back(*it);
		else rest.push_back(*it);
	}
}

void transform(std::vector<point>& cloud, std::vector<std::vector<long double>>& transformation_mat)
{
	int n=int(cloud.size());
	for(int i=0; i<n; ++i)
	{
		point temp=cloud[i];
		temp.x=transformation_mat[0][0]*cloud[i].x+transformation_mat[0][1]*cloud[i].y+transformation_mat[0][2]*cloud[i].z+transformation_mat[0][3]*1.0;
		temp.y=transformation_mat[1][0]*cloud[i].x+transformation_mat[1][1]*cloud[i].y+transformation_mat[1][2]*cloud[i].z+transformation_mat[1][3]*1.0;
		temp.z=transformation_mat[2][0]*cloud[i].x+transformation_mat[2][1]*cloud[i].y+transformation_mat[2][2]*cloud[i].z+transformation_mat[2][3]*1.0;
		cloud[i]=temp;
	}
}

void build_transform_normal(std::vector<std::vector<long double>>& transformation_mat, std::vector<long double>& normal1, std::vector<long double>& normal2)
{
	std::vector<long double> k(3);
	k[0] = normal1[1]*normal2[2]-normal1[2]*normal2[1];
	k[1]=-(normal1[0]*normal2[2]-normal1[2]*normal2[0]);
	k[2] = normal1[0]*normal2[1]-normal1[1]*normal2[0];
	long double norm = sqrt(k[0]*k[0]+k[1]*k[1]+k[2]*k[2]);
	for(int i=0; i<3; ++i) k[i]/=norm;
	
	long double ct=normal1[0]*normal2[0]+normal1[1]*normal2[1]+normal1[2]*normal2[2];
	long double st=sqrt(1-ct*ct);

	std::vector<std::vector<long double>> k_mat(3, std::vector<long double>(3));
	k_mat[0][0]=  0.0; k_mat[0][1]=-k[2]; k_mat[0][2]= k[1];
	k_mat[1][0]= k[2]; k_mat[1][1]=  0.0; k_mat[1][2]=-k[0];
	k_mat[2][0]=-k[1]; k_mat[2][1]= k[0]; k_mat[2][2]=  0.0;

	std::vector<std::vector<long double>> k_mat_square=mat_multi(k_mat, k_mat);
	
	std::vector<std::vector<long double>> identity_mat(3, std::vector<long double>(3, 0.0));
	for(int i=0; i<3; ++i) identity_mat[i][i]=1.0;
	
	std::vector<std::vector<long double>> rotation_mat(3, std::vector<long double>(3));
	
	for(int i=0; i<3; ++i)
	{
		for(int j=0; j<3; ++j)
		{
			rotation_mat[i][j]=identity_mat[i][j]+st*k_mat[i][j]+(1-ct)*k_mat_square[i][j];
		}
	}
	
	transformation_mat.clear();
	transformation_mat=std::vector<std::vector<long double>>(4, std::vector<long double>(4, 0.0));
	for(int i=0; i<3; ++i)
	{
		for(int j=0; j<3; ++j)
		{
			transformation_mat[i][j]=rotation_mat[i][j];
		}
	}
	transformation_mat[3][3]=1.0;
}

void build_transform_centroid(std::vector<std::vector<long double>>& transformation_mat, std::vector<long double>& normal1, std::vector<long double>& centroid1, std::vector<long double>& normal2, std::vector<long double>& centroid2)
{
	std::vector<long double> k(3);
	k[0] = normal1[1]*normal2[2]-normal1[2]*normal2[1];
	k[1]=-(normal1[0]*normal2[2]-normal1[2]*normal2[0]);
	k[2] = normal1[0]*normal2[1]-normal1[1]*normal2[0];
	long double norm = sqrt(k[0]*k[0]+k[1]*k[1]+k[2]*k[2]);
	for(int i=0; i<3; ++i) k[i]/=norm;
	
	long double ct=normal1[0]*normal2[0]+normal1[1]*normal2[1]+normal1[2]*normal2[2];
	long double st=sqrt(1-ct*ct);

	std::vector<std::vector<long double>> k_mat(3, std::vector<long double>(3));
	k_mat[0][0]=  0.0; k_mat[0][1]=-k[2]; k_mat[0][2]= k[1];
	k_mat[1][0]= k[2]; k_mat[1][1]=  0.0; k_mat[1][2]=-k[0];
	k_mat[2][0]=-k[1]; k_mat[2][1]= k[0]; k_mat[2][2]=  0.0;

	std::vector<std::vector<long double>> k_mat_square=mat_multi(k_mat, k_mat);
	
	std::vector<std::vector<long double>> identity_mat(3, std::vector<long double>(3, 0.0));
	for(int i=0; i<3; ++i) identity_mat[i][i]=1.0;
	
	std::vector<std::vector<long double>> rotation_mat(3, std::vector<long double>(3));
	
	for(int i=0; i<3; ++i)
	{
		for(int j=0; j<3; ++j)
		{
			rotation_mat[i][j]=identity_mat[i][j]+st*k_mat[i][j]+(1-ct)*k_mat_square[i][j];
		}
	}
	
	transformation_mat.clear();
	transformation_mat=std::vector<std::vector<long double>>(4, std::vector<long double>(4, 0.0));
	for(int i=0; i<3; ++i)
	{
		for(int j=0; j<3; ++j)
		{
			transformation_mat[i][j]=rotation_mat[i][j];
		}
	}
	transformation_mat[2][3]=centroid2[2]-(rotation_mat[2][0]*centroid1[0]+rotation_mat[2][1]*centroid1[1]+rotation_mat[2][2]*centroid1[2]);
	transformation_mat[3][3]=1.0;
}

void build_transform(std::vector<std::vector<long double>>& transformation_mat, long double theta, long double x, long double y)
{
	transformation_mat.clear();
	transformation_mat=std::vector<std::vector<long double>>(4, std::vector<long double>(4, 0.0));
	transformation_mat[0][0]=cos(theta); transformation_mat[0][1]=-sin(theta); transformation_mat[0][3]=x;
	transformation_mat[1][0]=sin(theta); transformation_mat[1][1]=cos(theta); transformation_mat[1][3]=y;
	transformation_mat[2][2]=1.0;
	transformation_mat[3][3]=1.0;
}
