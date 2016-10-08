#include <mireg.hpp>
#include <project_classes.hpp>
#include <pcl_visualizer.hpp>
#include <transformation.hpp>
#include <characteristic_map.hpp>
#include <mutual_information.hpp>
#include <registration.hpp>

void fill_sample_space(std::vector<Eigen::Vector3d>& states, int n_random_states, double step, double dist_to_angle, double dist_to_prob_dist)
{
	states.clear(); states.reserve(50+n_random_states);

	Eigen::Vector3d temp;
	
	// uniformly selected 26 states
	temp=Eigen::Vector3d(0.0, 0.0, step*dist_to_angle); states.push_back(temp);
	temp=Eigen::Vector3d(0.0, 0.0, -step*dist_to_angle); states.push_back(temp);
	temp=Eigen::Vector3d(step, 0.0, 0.0); states.push_back(temp);
	temp=Eigen::Vector3d(-step, 0.0, 0.0); states.push_back(temp);
	temp=Eigen::Vector3d(0.0, step, 0.0); states.push_back(temp);
	temp=Eigen::Vector3d(0.0, -step, 0.0); states.push_back(temp);

	temp=Eigen::Vector3d(step, 0.0, step*dist_to_angle); states.push_back(temp);
	temp=Eigen::Vector3d(-step, 0.0, step*dist_to_angle); states.push_back(temp);
	temp=Eigen::Vector3d(step, 0.0, -step*dist_to_angle); states.push_back(temp);
	temp=Eigen::Vector3d(-step, 0.0, -step*dist_to_angle); states.push_back(temp);
	temp=Eigen::Vector3d(0.0, step, step*dist_to_angle); states.push_back(temp);
	temp=Eigen::Vector3d(0.0, -step, step*dist_to_angle); states.push_back(temp);
	temp=Eigen::Vector3d(0.0, step, -step*dist_to_angle); states.push_back(temp);
	temp=Eigen::Vector3d(0.0, -step, -step*dist_to_angle); states.push_back(temp);
	temp=Eigen::Vector3d(step, step, 0.0); states.push_back(temp);
	temp=Eigen::Vector3d(step, -step, 0.0); states.push_back(temp);
	temp=Eigen::Vector3d(-step, step, 0.0); states.push_back(temp);
	temp=Eigen::Vector3d(-step, -step, 0.0); states.push_back(temp);

	temp=Eigen::Vector3d(step, step, step*dist_to_angle); states.push_back(temp);
	temp=Eigen::Vector3d(step, -step, step*dist_to_angle); states.push_back(temp);
	temp=Eigen::Vector3d(-step, step, step*dist_to_angle); states.push_back(temp);
	temp=Eigen::Vector3d(-step, -step, step*dist_to_angle); states.push_back(temp);
	temp=Eigen::Vector3d(step, step, -step*dist_to_angle); states.push_back(temp);
	temp=Eigen::Vector3d(step, -step, -step*dist_to_angle); states.push_back(temp);
	temp=Eigen::Vector3d(-step, step, -step*dist_to_angle); states.push_back(temp);
	temp=Eigen::Vector3d(-step, -step, -step*dist_to_angle); states.push_back(temp);
												
	// randomly selected states
	std::random_device rd;
	std::mt19937 eng(rd());
	std::uniform_real_distribution<> distr(0.0, step*dist_to_prob_dist);
	for(int i=0; i<n_random_states; ++i)
	{
		temp=Eigen::Vector3d(distr(eng), distr(eng), distr(eng)*dist_to_angle);
		states.push_back(temp);
	}
}

bool multires_registration(std::vector<point>& reading, std::vector<point>& reference, Eigen::Matrix4d& transformation_mat, double min_cell_size, int hist_size, std::string& map)
{
		//variance_map, reflectivity_map, grayscale_map, normals_map
		std::vector<double> map_weightage_coeff(4);
		map_weightage_coeff[0]=7.0; map_weightage_coeff[1]=1.0; map_weightage_coeff[2]=1.0; map_weightage_coeff[3]=9.0;
		
		typedef void (*characteristic_map_func)(std::vector<point>&, double, std::vector<std::pair<pii, double>>&);
		std::vector<characteristic_map_func> feature_map_func;
		if(map=="all")
		{
			feature_map_func.push_back(&variance_map);
			feature_map_func.push_back(&reflectivity_map);
			feature_map_func.push_back(&grayscale_map);
			feature_map_func.push_back(&normal_map);
		}
		else if(map=="variance") feature_map_func.push_back(&variance_map);
		else if(map=="reflectivity") feature_map_func.push_back(&reflectivity_map);
		else if(map=="grayscale") feature_map_func.push_back(&grayscale_map);
		else if(map=="normal") feature_map_func.push_back(&normal_map);
		else return false;
				
		double dist_to_angle=0.05235987756, dist_to_prob_dist=5.0;	// fine tune dist_to_prob_dist
		int n_random_states=0;	// add later
		double step_ratio=2.0, cell_reduction_constant=2.0;
		
		double cell_size=3.0;
		double step_coeff=2.0;

		Eigen::Vector3d best_transformation(0.0, 0.0, 0.0);
		while(cell_size>=min_cell_size/2.0)
		{
			double step=cell_size*step_coeff;
//			double smallest=cell_size/(2.0*dist_to_prob_dist);
			double smallest=cell_size/4.0;
			
			std::vector<point> r(reading);
			build_transform_z(transformation_mat, best_transformation);
			transform(r, transformation_mat);
			ld best_mi=0.0;
			for(int mit=0; mit<int(feature_map_func.size()); ++mit)
			{
				std::vector<std::pair<pii, double>> feature_map1, feature_map2;
				feature_map_func[mit](r, cell_size, feature_map1);
				feature_map_func[mit](reference, cell_size, feature_map2);
				double temp=map_weightage_coeff[mit]*mutual_information(feature_map1, feature_map2, hist_size);
				best_mi+=temp;
			}
			while(step>smallest)
			{
				std::vector<Eigen::Vector3d> states;
			
				fill_sample_space(states, n_random_states, step, dist_to_angle, dist_to_prob_dist);
			
				ld better_mi=best_mi;
				Eigen::Vector3d better_transformation(0.0, 0.0, 0.0);
				int n_selection=-1;
				for(int i=0; i<int(states.size()); ++i)
				{
					r.clear();
					r=reading;
					Eigen::Vector3d new_transformation_state = best_transformation + states[i];
					build_transform_z(transformation_mat, new_transformation_state);
					transform(r, transformation_mat);
					ld mi=0.0;
					for(int mit=0; mit<int(feature_map_func.size()); ++mit)
					{
						std::vector<std::pair<pii, double>> feature_map1, feature_map2;
						feature_map_func[mit](r, cell_size, feature_map1);
						feature_map_func[mit](reference, cell_size, feature_map2);
						double temp=map_weightage_coeff[mit]*mutual_information(feature_map1, feature_map2, hist_size);
						mi+=temp;
					}
					if(mi>better_mi)
					{
						better_mi=mi;
						better_transformation=best_transformation+states[i];
						n_selection=i;
					}
				}
				if(n_selection!=-1)
				{
					best_mi=better_mi;
					best_transformation=better_transformation;
					if(n_selection>=int(states.size()-n_random_states))
					{
						if(states[n_selection](0)>step) step=states[n_selection](0);
					}
					step*=step_ratio;
				}
				else
				{
					step/=step_ratio;
				}
			}
			cell_size/=cell_reduction_constant;
			std::cout<<"cell_size: "<<cell_size<<", cells_best_mi: "<<best_mi<<std::endl;
		}
		build_transform_z(transformation_mat, best_transformation);
		transform(reading, transformation_mat);
		return true;
}
