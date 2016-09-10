#include <mireg.h>
#include <pcl_visualizer.h>
#include <pcl_helper.h>
#include <transformation.h>
#include <registration.h>

//const float min_cell_size = 0.1;	// in metres
//const int hist_size = 100;	// sampling size for histogram

const float min_cell_size = 0.1;	// in metres
const int hist_size = 100;	// sampling size for histogram

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

bool file_write(std::string filename, std::vector<long double>& euler_rep)
{
	std::ofstream file(filename);
	if(file.is_open())
	{
		for(int i=0; i<int(euler_rep.size()); ++i)
		{
			file<<euler_rep[i]<<" ";
		}
		file<<std::endl;
		return true;
	}
	else
	{
		return false;
	}
}

void print_transform(std::vector<std::vector<long double>>& transformation_mat)
{
	for(int i=0; i<4; ++i)
	{
		for(int j=0; j<4; ++j)
		{
			std::cout<<transformation_mat[i][j]<<"\t\t";
		}
		std::cout<<std::endl;
	}
}

void convert(std::vector<long double>& eu, std::vector<std::vector<long double>>& transform)
{
	long double x=eu[0], y=eu[1], z=eu[2], thetax=eu[3], thetay=eu[4], thetaz=eu[5];
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
		temp.x=transform[0][0]*scan[i].x+transform[0][1]*scan[i].y+transform[0][2]*scan[i].z+transform[0][3];
		temp.y=transform[1][0]*scan[i].x+transform[1][1]*scan[i].y+transform[1][2]*scan[i].z+transform[1][3];
		temp.z=transform[2][0]*scan[i].x+transform[2][1]*scan[i].y+transform[2][2]*scan[i].z+transform[2][3];
		scan[i]=temp;
	}
}

int main(int argc, char** argv)
{
	if(argc!=10)
	{
		std::cout<<"usage: ./gicp3 file_name1 file_name2 x y z thetax thetay thetaz input_number\n";
	}
	else
	{
		std::vector<point> reading, reference;
		if(!file_read(argv[1], reading))
		{
			std::cout<<"File read failed!!!"; exit(1);
		}
		if(!file_read(argv[2], reference))
		{
			std::cout<<"File read failed!!!"; exit(1);	// write these error messages to the output file
		}
//		plot_merged(reading, reference);
		
		long double x=std::stold(argv[3]), y=std::stold(argv[4]), z=std::stold(argv[5]), thetax=std::stold(argv[6]), thetay=std::stold(argv[7]), thetaz=std::stold(argv[8]);
		std::vector<std::vector<long double>> t(4, std::vector<long double>(4, 0.0));
		std::vector<long double> eu(6); eu[0]=x; eu[1]=y; eu[2]=z; eu[3]=thetax; eu[4]=thetay; eu[5]=thetaz;
		convert(eu, t);
		process(reading, t);		
		
//		plot_merged(reading, reference);
		
		std::vector<std::vector<long double>> transformation_mat1, transformation_mat2, transformation_mat3;
		std::vector<point> ground1, ground2, rest1, rest2;
		std::vector<long double> normal1, normal2, centroid1, centroid2;
		std::vector<long double> z_axis(3, 0.0); z_axis[2]=1.0;
		std::vector<long double> origin(3, 0.0);

		
		// Find the transform to align reference plane to plane z=0 and apply
		ground_plane_extraction(reference, ground2, rest2);
		best_fit_plane1(ground2, normal2, centroid2);	// best_fit_plane
		build_transform_normal(transformation_mat1, normal2, z_axis);
		transform(reading, transformation_mat1); transform(reference, transformation_mat1);
		
		ground_plane_extraction(reference, ground2, rest2);
		best_fit_plane1(ground2, normal2, centroid2);
		std::vector<std::vector<long double>> new_transformation_mat1;
		build_transform_centroid(new_transformation_mat1, normal2, centroid2, z_axis, origin);
		transform(reading, new_transformation_mat1); transform(reference, new_transformation_mat1);
		transformation_mat1=mat_multi(new_transformation_mat1, transformation_mat1);
		
		// Find the transform to align reading plane to plane z=0 and apply
		ground_plane_extraction(reading, ground1, rest1);
		best_fit_plane1(ground1, normal1, centroid1);	// best_fit_plane
		build_transform_normal(transformation_mat2, normal1, z_axis);
		transform(reading, transformation_mat2);
		
		ground_plane_extraction(reading, ground1, rest1);
		best_fit_plane1(ground1, normal1, centroid1);
		std::vector<std::vector<long double>> new_transformation_mat2;
		build_transform_centroid(new_transformation_mat2, normal1, centroid1, z_axis, origin);
		transform(reading, new_transformation_mat2);
		transformation_mat2=mat_multi(new_transformation_mat2, transformation_mat2);
		
//		plot_merged(reading, reference);
		std::string map("all");	// change it to all
		if(!multires_registration(reading, reference, transformation_mat3, min_cell_size, hist_size, map))
		{
			std::cout<<"Incorrect 3rd argument passed"<<std::endl;
			return 1;
		}
		
		std::vector<std::vector<long double>> transformation_mat, temp;
		temp=mat_inv(transformation_mat1);
		temp=mat_multi(temp, transformation_mat3);
		temp=mat_multi(temp, transformation_mat2);
		transformation_mat=mat_multi(temp, transformation_mat1);
		transformation_mat=mat_multi(transformation_mat, t);
		
		std::string output_name(map); output_name.push_back('_');

		std::string f1(argv[1]), f2(argv[2]);

		std::string s1, s2;
		s1.push_back(f1[f1.size()-8]); s1.push_back(f1[f1.size()-7]); s1.push_back(f1[f1.size()-6]); s1.push_back(f1[f1.size()-5]);
		output_name.append(s1);
		output_name.push_back('_');
		s2.push_back(f2[f2.size()-8]); s2.push_back(f2[f2.size()-7]); s2.push_back(f2[f2.size()-6]); s2.push_back(f2[f2.size()-5]);
		output_name.append(s2);
		
		std::string input_number(argv[9]);
		output_name.push_back('_'); output_name.append(input_number);
		output_name.append("_result.txt");
		
//		file_write(output_name, transformation_mat);
		
		std::vector<long double> ans = euler_rep(transformation_mat);
		/*
		for(int i=0; i<6; ++i)
		{
			std::cout<<ans[i]<<" ";
		}
		std::cout<<std::endl;
		*/
		file_write(output_name, ans);
		std::cout<<map<<" "<<s1<<" "<<s2<<" "<<input_number<<" "<<x<<" "<<y<<" "<<z<<" "<<thetax<<" "<<thetay<<" "<<thetaz<<" "<<ans[0]<<" "<<ans[1]<<" "<<ans[2]<<" "<<ans[3]<<" "<<ans[4]<<" "<<ans[5]<<std::endl;
//		plot_merged(reading, reference);
	}
	return 0;
}
