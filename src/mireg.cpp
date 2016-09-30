#include <mireg.hpp>
#include <pcl_visualizer.hpp>
#include <pcl_helper.hpp>
#include <transformation.hpp>
#include <registration.hpp>

const double min_cell_size = 0.1;	// in metres
const int hist_size = 100;	// sampling size for histogram

bool file_read(std::string filename, std::vector<point>& cloud)
{
	std::ifstream file(filename);
	cloud.clear(); cloud.reserve(1e5);
	if(file.is_open())
	{
		double frame; file>>frame;
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

bool file_write(std::string filename, std::vector<double>& euler_rep)
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

int main(int argc, char** argv)
{
	if(argc!=4)
	{
		std::cout<<"usage: ./mireg file_name1 file_name2 {all, variance, reflectivity, grayscale, normal}\n";
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

//		ORIGINAL
//		plot_merged(reading, reference);
		Eigen::Vector3d z_axis(0, 0, 1), origin(0, 0, 0);

		std::vector<point> ground2, rest2;
		ground_plane_extraction(reference, ground2, rest2);	// Find the transform to align reference plane to plane z=0 and apply		
		Eigen::Vector3d normal2, centroid2;
		best_fit_plane1(ground2, normal2, centroid2);	// best_fit_plane
		Eigen::Matrix4d transformation_mat1;
		build_transform_xy(transformation_mat1, normal2, centroid2, z_axis, origin);
		transform(reading, transformation_mat1); transform(reference, transformation_mat1);

		ground_plane_extraction(reference, ground2, rest2);
		best_fit_plane1(ground2, normal2, centroid2);
		Eigen::Matrix4d new_transformation_mat1;
		build_transform_xy(new_transformation_mat1, normal2, centroid2, z_axis, origin);
		transform(reading, new_transformation_mat1); transform(reference, new_transformation_mat1);

		transformation_mat1 = new_transformation_mat1 * transformation_mat1;
		
		std::vector<point> ground1, rest1;
		ground_plane_extraction(reading, ground1, rest1);	// Find the transform to align reading plane to plane z=0 and apply
		Eigen::Vector3d normal1, centroid1;
		best_fit_plane1(ground1, normal1, centroid1);
		Eigen::Matrix4d transformation_mat2;
		build_transform_xy(transformation_mat2, normal1, centroid1, z_axis, origin);
		transform(reading, transformation_mat2);

		ground_plane_extraction(reading, ground1, rest1);
		best_fit_plane1(ground1, normal1, centroid1);
		Eigen::Matrix4d new_transformation_mat2;
		build_transform_xy(new_transformation_mat2, normal1, centroid1,  z_axis, origin);
		transform(reading, new_transformation_mat2);

		transformation_mat2 = new_transformation_mat2 * transformation_mat2;

		// GROUND PLANES ALLIGNED
//		plot_merged(reading, reference);

		std::string map(argv[3]);
		Eigen::Matrix4d transformation_mat3;
		if(!multires_registration(reading, reference, transformation_mat3, min_cell_size, hist_size, map))
		{
			std::cout<<"Incorrect 3rd argument passed"<<std::endl;
			return 1;
		}		

		Eigen::Matrix4d transformation_mat;
		transformation_mat=transformation_mat1.inverse()*transformation_mat3*transformation_mat2*transformation_mat1;

		std::string output_name(map); output_name.push_back('_');	// Can try regular expressions
		std::string f1(argv[1]), f2(argv[2]);
		std::string s1, s2;
		s1.push_back(f1[f1.size()-8]); s1.push_back(f1[f1.size()-7]); s1.push_back(f1[f1.size()-6]); s1.push_back(f1[f1.size()-5]);
		output_name.append(s1);
		output_name.push_back('_');
		s2.push_back(f2[f2.size()-8]); s2.push_back(f2[f2.size()-7]); s2.push_back(f2[f2.size()-6]); s2.push_back(f2[f2.size()-5]);
		output_name.append(s2);
		output_name.append("_result.txt");

		std::vector<double> ans = euler_rep(transformation_mat);	// Use Eigen implementation to calculate euler_rep
		file_write(output_name, ans);
		std::cout<<map<<" "<<s1<<" "<<s2<<" "<<ans[0]<<" "<<ans[1]<<" "<<ans[2]<<" "<<ans[3]<<" "<<ans[4]<<" "<<ans[5]<<std::endl;

		// RESULT
//		plot_merged(reading, reference);
	}
	return 0;
}
