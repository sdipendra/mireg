#include <mireg.hpp>
#include <project_classes.hpp>

bool scan_read(std::string filename, std::vector<point>& cloud)
{
	std::ifstream file(filename);
	cloud.clear(); cloud.reserve(1e5);
	if(file.is_open())
	{
		float frame; file>>frame;
		point temp; temp.get_point(file);
		while(!file.eof())
		{
			cloud.push_back(temp);
			temp.get_point(file);
		}
		file.close();
		return true;
	}
	else
	{
		return false;
	}
}

bool scan_write(std::string filename, std::vector<point>& cloud)
{
	std::ofstream file(filename);
	if(file.is_open())
	{
		file<<cloud.size()<<std::endl;
		for(int i=0; i<int(cloud.size()); ++i)
		{
			file<<cloud[i].coordinate(0)<<" "<<cloud[i].coordinate(1)<<" "<<cloud[i].coordinate(2)<<" "<<cloud[i].normal(0)<<" "<<cloud[i].normal(1)<<" "<<cloud[i].normal(2)<<" "<<cloud[i].ref<<" "<<cloud[i].gray<<std::endl;
		}
		file.close();
		return true;
	}
	else
	{
		return false;
	}
}

void process(std::vector<point>& scan, Eigen::Matrix4d& transformation_mat)
{
	int n=scan.size();
	Eigen::Transform<double, 3, Eigen::Affine> t(transformation_mat);
	for(int i=0; i<n; ++i)
	{
		scan[i].coordinate=t*scan[i].coordinate;	// Can also consider to transform normals
	}
}

int main( int argc, char** argv)
{
	if(argc != 8)
	{
		std::cout<<"usage: ./transformer filename_scan theta_x theta_y theta_z x y z"<<std::endl;
	}
	else
	{
		std::vector<point> scan;
		if(!scan_read(argv[1], scan))
		{
			std::cout<<"File Read Failed!! Exiting.."<<std::endl;
			exit(1);
		}
		
		double thetax, thetay, thetaz, x, y, z;
		thetax = std::stod(argv[2]); thetay = std::stod(argv[3]); thetaz = std::stod(argv[4]);
		thetax*=(pi/180.0); thetay*=(pi/180.0);	thetaz*=(pi/180.0);
		x=std::stod(argv[5]); y=std::stod(argv[6]); z=std::stod(argv[7]);

		Eigen::Matrix4d transform;
		transform=Eigen::Matrix4d::Identity();
		transform(0, 0)=cos(thetay)*cos(thetaz);
		transform(0, 1)=sin(thetax)*sin(thetay)*cos(thetaz)-cos(thetax)*sin(thetaz);
		transform(0, 2)=cos(thetax)*sin(thetay)*cos(thetaz)+sin(thetax)*sin(thetaz);
		transform(0, 3)=x;
		
		transform(1, 0)=cos(thetay)*sin(thetaz);
		transform(1, 1)=sin(thetax)*sin(thetay)*sin(thetaz)+cos(thetax)*cos(thetaz);
		transform(1, 2)=cos(thetax)*sin(thetay)*sin(thetaz)-sin(thetax)*cos(thetaz);
		transform(1, 3)=y;
		
		transform(2, 0)=-sin(thetay);
		transform(2, 1)=sin(thetax)*cos(thetay);
		transform(2, 2)=cos(thetax)*cos(thetay);
		transform(2, 3)=z;
		
		transform(3, 3)=1.0;
		
		process(scan, transform);
		
		std::string filename("T"); filename.append(argv[1]);
		scan_write(filename, scan);
	}
	return 0;
}
