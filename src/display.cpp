#include <mireg.hpp>
#include <project_classes.hpp>
#include <pcl_visualizer.hpp>

bool file_read(std::string filename, std::vector<point>& cloud)
{
	std::ifstream file(filename);
	cloud.clear();
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

bool transform_read(std::string filename, std::vector<long double>& euler_rep)
{
	std::ifstream file(filename);
	if(file.is_open())
	{
		for(int i=0; i<6; ++i) file>>euler_rep[i];
		file.close();
		return true;
	}
	else
	{
		return false;
	}
}

void euler_to_matrix(std::vector<long double>& euler_rep, std::vector<std::vector<long double>>& transform)
{
	long double x=euler_rep[0], y=euler_rep[1], z=euler_rep[2], thetax=euler_rep[3], thetay=euler_rep[4], thetaz=euler_rep[5];
	thetax*=(pi/180.0); thetay*=(pi/180.0);	thetaz*=(pi/180.0);

	
	transform[0][0]=cos(thetay)*cos(thetaz);
	transform[0][1]=sin(thetax)*sin(thetay)*cos(thetaz)-cos(thetax)*sin(thetaz);
	transform[0][2]=cos(thetax)*sin(thetay)*cos(thetaz)+sin(thetax)*sin(thetaz);
	
	transform[1][0]=cos(thetay)*sin(thetaz);
	transform[1][1]=sin(thetax)*sin(thetay)*sin(thetaz)+cos(thetax)*cos(thetaz);
	transform[1][2]=cos(thetax)*sin(thetay)*sin(thetaz)-sin(thetax)*cos(thetaz);
	
	transform[2][0]=-sin(thetay);
	transform[2][1]=sin(thetax)*cos(thetay);
	transform[2][2]=cos(thetax)*cos(thetay);
	
	transform[0][3]=x;
	transform[1][3]=y;
	transform[2][3]=z;
	transform[3][3]=1.0;

}

void process(std::vector<point>& scan, std::vector<std::vector<long double>>& transform)
{
	for(int i=0; i<int(scan.size()); ++i)
	{
		point temp=scan[i];
		temp.coordinate(0)=transform[0][0]*scan[i].coordinate(0)+transform[0][1]*scan[i].coordinate(1)+transform[0][2]*scan[i].coordinate(2)+transform[0][3];
		temp.coordinate(1)=transform[1][0]*scan[i].coordinate(0)+transform[1][1]*scan[i].coordinate(1)+transform[1][2]*scan[i].coordinate(2)+transform[1][3];
		temp.coordinate(2)=transform[2][0]*scan[i].coordinate(0)+transform[2][1]*scan[i].coordinate(1)+transform[2][2]*scan[i].coordinate(2)+transform[2][3];
		scan[i]=temp;
	}
}

int main(int argc, char** argv)
{
	if(argc==2)
	{
		std::vector<point> cloud;
		if(!file_read(argv[1], cloud))
		{
			std::cout<<"File read failed!!!";
			exit(1);
		}
		vvd points;
		for(auto it=cloud.begin(); it!=cloud.end(); ++it)
		{
			vd temp(3);
			temp[0]=it->coordinate(0); temp[1]=it->coordinate(1); temp[2]=it->coordinate(2);
			points.push_back(temp);
		}
		plot(points);
	}
	else if(argc==3)
	{
		std::vector<point> reading, reference;
		if(!file_read(argv[1], reading))
		{
			std::cout<<"File read failed!!!"; exit(1);
		}
		if(!file_read(argv[2], reference))
		{
			std::cout<<"File read failed!!!"; exit(1);
		}
		plot_merged(reading, reference);
	}
	else if(argc==4)
	{
		std::vector<point> reading, reference;
		std::vector<long double> euler_rep(6, 0.0);
		std::vector<std::vector<long double>> transform(4, std::vector<long double>(4, 0.0));
		if(!file_read(argv[1], reading))
		{
			std::cout<<"File read failed!!!"; exit(1);
		}
		if(!file_read(argv[2], reference))
		{
			std::cout<<"File read failed!!!"; exit(1);
		}
		if(!transform_read(argv[3], euler_rep))
		{
			std::cout<<"File Read Failed!! Exiting.."<<std::endl; exit(1);
		}
		euler_to_matrix(euler_rep, transform);
		process(reading, transform);
		plot_merged(reading, reference);
	}
	else
	{
		std::cout<<"usage:"<<std::endl<<"./display file_name"<<std::endl<<"./display file_name1 file_name2"<<std::endl<<"./display file_name1 file_name2 transform_file"<<std::endl;
	}
	return 0;
}
