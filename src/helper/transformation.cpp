#include <project_classes.hpp>
#include <transformation.hpp>

const int grid_length=100, grid_breadth=100;
const double clearance=0.1;

bool equal(const double& n1, const double& n2, double epsilon=std::numeric_limits<double>::epsilon())
{
	return(epsilon>std::abs(n1-n2));
}

std::vector<double> euler_rep(Eigen::Matrix4d& transformation_mat)
{
	std::vector<double> ans(6);
	
	for(int i=0; i<3; ++i) ans[i]=transformation_mat(i, 3);	// translations
	
	// check for gimbal locking
	if(equal(transformation_mat(2, 0), (double)-1.0))
	{
		ans[3] = pi/2.0;
		ans[4] = 0.0;
		ans[5] = ans[3]+atan2(transformation_mat(0, 1), transformation_mat(0, 2));
	}
	else if(equal(transformation_mat(2, 0), (double)1.0))
	{
		ans[3] = -pi/2.0;
		ans[4] = 0.0;
		ans[5] = -ans[3] + atan2(-transformation_mat(0, 1), -transformation_mat(0, 2));
	}
	else
	{
		double y1 = -asin(transformation_mat(2, 0));
		double y2 = pi - y1;
		
		double x1 = atan2(transformation_mat(2, 1) / cos(y1), transformation_mat(2, 2) / cos(y1));
		double x2 = atan2(transformation_mat(2, 1) / cos(y2), transformation_mat(2, 2) / cos(y2));
		
		double z1 = atan2(transformation_mat(1, 0) / cos(y1), transformation_mat(0, 0) / cos(y1));
		double z2 = atan2(transformation_mat(1, 0) / cos(y2), transformation_mat(0, 0) / cos(y2));
		
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
	double maxx=-std::numeric_limits<double>::max(), maxy=-std::numeric_limits<double>::max(), maxz=-std::numeric_limits<double>::max(), minx=std::numeric_limits<double>::max(), miny=std::numeric_limits<double>::max(), minz=std::numeric_limits<double>::max();
	for(auto it=complete.begin(); it!=complete.end(); ++it)
	{
		if(it->coordinate(0)>maxx) maxx=it->coordinate(0);
		if(it->coordinate(1)>maxy) maxy=it->coordinate(1);
		if(it->coordinate(2)>maxz) maxz=it->coordinate(2);
		
		if(it->coordinate(0)<minx) minx=it->coordinate(0);
		if(it->coordinate(1)<miny) miny=it->coordinate(1);
		if(it->coordinate(2)<minz) minz=it->coordinate(2);
	}
	
	double length=maxx-minx, breadth=maxy-miny, height=maxz-minz;
	double xbox=(length)/(grid_length-1), ybox=(breadth)/(grid_breadth-1);
	vvd grid(grid_length, vd(grid_breadth, std::numeric_limits<double>::max()));
	for(auto it=complete.begin(); it!=complete.end(); ++it)
	{
		int x=(it->coordinate(0)-minx+xbox/2.0)/xbox, y=(it->coordinate(1)-miny+ybox/2.0)/ybox;
		if(grid[x][y]>it->coordinate(2)) grid[x][y]=it->coordinate(2);
	}
	
	ground.clear(); rest.clear();
	for(auto it=complete.begin(); it!=complete.end(); ++it)
	{
		int x=(it->coordinate(0)-minx+xbox/2.0)/xbox, y=(it->coordinate(1)-miny+ybox/2.0)/ybox;
		if(it->coordinate(2)-grid[x][y]<clearance) ground.push_back(*it);
		else rest.push_back(*it);
	}
}

void transform(std::vector<point>& cloud, Eigen::Matrix4d& transformation_mat)
{
	int n=int(cloud.size());
	Eigen::Transform<double, 3, Eigen::Affine> t(transformation_mat);
	for(int i=0; i<n; ++i)
	{
		cloud[i].coordinate=t*cloud[i].coordinate;
	}
}

void build_transform_xy(Eigen::Matrix4d& transformation_mat, Eigen::Vector3d& normal1, Eigen::Vector3d& centroid1, Eigen::Vector3d& normal2, Eigen::Vector3d& centroid2)
{
	normal1.normalize(); normal2.normalize();
	Eigen::Vector3d k;
	k = normal1.cross(normal2); k.normalize();
	double angle=acos(normal1.dot(normal2));
	Eigen::Matrix3d rotation_mat;
	rotation_mat=Eigen::AngleAxisd(angle, k);
	transformation_mat=Eigen::Matrix4d::Identity();
	transformation_mat.block<3, 3>(0, 0) = rotation_mat;
	transformation_mat.block<3, 1>(0, 3) = centroid2-rotation_mat*centroid1;
}

void build_transform_z(Eigen::Matrix4d& transformation_mat, Eigen::Vector3d& euler)	// remove using euler to matrix rep
{
	transformation_mat=Eigen::Matrix4d::Identity();
	transformation_mat(0, 0)=cos(euler(2)); transformation_mat(0, 1)=-sin(euler(2));
	transformation_mat(1, 0)=sin(euler(2)); transformation_mat(1, 1)=cos(euler(2));
	transformation_mat(0, 3)=euler(0);
	transformation_mat(1, 3)=euler(1);
}
