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

bool transform_write(std::string filename, std::vector<std::vector<long double>>& transformation_mat)
{
	std::ofstream file(filename);
	if(file.is_open())
	{
		for(int i=0; i<4; ++i)
		{
			for(int j=0; j<4; ++j)
			{
				file<<transformation_mat[i][j]<<" ";
			}
			file<<std::endl;
		}
		file.close();
		return true;
	}
	else
	{
		return false;
	}
}

int main( int argc, char** argv)
{
	if(argc != 7)
	{
		std::cout<<"usage: ./generate_transformation x y z theta_x theta_y theta_z\n";
	}
	else
	{
		std::vector<std::vector<long double>> transform(4, std::vector<long double>(4, 0.0));
		long double x, y, z, thetax, thetay, thetaz;
		x=std::stold(argv[1]); y=std::stold(argv[2]); z=std::stold(argv[3]);
		thetax = std::stold(argv[4]); thetay = std::stold(argv[5]); thetaz = std::stold(argv[6]);
		thetax*=(pi/180.0); thetay*=(pi/180.0);	thetaz*=(pi/180.0);

		// TODO: Write the formula for conversion of euler angles to rotation matrix
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
		
		std::string filename("t.txt");
		transform_write(filename, transform);
	}
	return 0;
}
