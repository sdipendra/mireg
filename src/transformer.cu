#include <mireg.h>

bool transform_read(std::string filename, std::vector<std::vector<long double>>& transform_mat)
{
	std::ifstream file(filename);
	if(file.is_open())
	{
		for(int i=0; i<4; ++i) for(int j=0; j<4; ++j) file>>transform_mat[i][j];
		file.close();
		return true;
	}
	else
	{
		return false;
	}
}

bool scan_read(std::string filename, std::vector<point>& cloud)
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

bool scan_write(std::string filename, std::vector<point>& cloud)
{
	std::ofstream file(filename);
	if(file.is_open())
	{
		file<<cloud.size()<<"\n";
		for(int i=0; i<int(cloud.size()); ++i)
		{
			file<<cloud[i].x<<" "<<cloud[i].y<<" "<<cloud[i].z<<" "<<cloud[i].nx<<" "<<cloud[i].ny<<" "<<cloud[i].nz<<" "<<cloud[i].ref<<" "<<cloud[i].gray<<std::endl;
		}
		file.close();
		return true;
	}
	else
	{
		return false;
	}
}

void process(std::vector<point>& scan, std::vector<std::vector<long double>>& transform)
{
	for(int i=0; i<int(scan.size()); ++i)
	{
		point temp=scan[i];
		temp.x=transform[0][0]*scan[i].x+transform[0][1]*scan[i].y+transform[0][2]*scan[i].z+transform[0][3];
		temp.y=transform[1][0]*scan[i].x+transform[1][1]*scan[i].y+transform[1][2]*scan[i].z+transform[1][3];
		temp.z=transform[2][0]*scan[i].x+transform[2][1]*scan[i].y+transform[2][2]*scan[i].z+transform[2][3];
		scan[i]=temp;
	}
}


int main( int argc, char** argv)
{
	if(argc != 3)
	{
		std::cout<<"usage: ./transformer filename_scan filename_transformation\n";
	}
	else
	{
		std::vector<point> scan;
		std::vector<std::vector<long double>> transform(4, std::vector<long double>(4));
		if(!scan_read(argv[1], scan))
		{
			std::cout<<"File Read Failed!! Exiting.."<<std::endl;
			exit(1);
		}
		if(!transform_read(argv[2], transform))
		{
			std::cout<<"File Read Failed!! Exiting.."<<std::endl;
			exit(1);
		}
		
		process(scan, transform);
		
		std::string filename(argv[1]);
		filename.pop_back(); filename.pop_back(); filename.pop_back(); filename.pop_back();
		filename.append("T.txt");
		scan_write(filename, scan);
	}
	return 0;
}
