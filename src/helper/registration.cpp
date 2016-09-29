#include <mireg.hpp>
#include <project_classes.hpp>
#include <pcl_visualizer.hpp>
#include <transformation.hpp>
#include <characteristic_map.hpp>
#include <mutual_information.hpp>
#include <registration.hpp>

typedef void (*characteristic_map_func)(std::vector<point>&, float, std::pair<pii, vvf>&);

bool multires_registration(std::vector<point>& reading, std::vector<point>& reference, std::vector<std::vector<long double>>& transformation_mat, float min_cell_size, int hist_size, std::string& map)
{
//		void (*feature_map_func)(std::vector<point>&, float, std::pair<pii, vvf>&) = &variance_map;	//variance_map, reflectivity_map, grayscale_map, normals_map
		std::vector<long double> map_weightage_coeff(4);
		map_weightage_coeff[0]=7.0; map_weightage_coeff[1]=1.0; map_weightage_coeff[2]=1.0; map_weightage_coeff[3]=9.0;	// tune these values, and build correspondence even if only one feature_map_func is used
		std::vector<characteristic_map_func> feature_map_func;
		if(map=="all")
		{
			feature_map_func.push_back(&variance_map);
			feature_map_func.push_back(&reflectivity_map);
			feature_map_func.push_back(&grayscale_map);
			feature_map_func.push_back(&normal_map);
		}
		else if(map=="variance")
		{
			feature_map_func.push_back(&variance_map);
		}
		else if(map=="reflectivity")
		{
			feature_map_func.push_back(&reflectivity_map);
		}
		else if(map=="grayscale")
		{
			feature_map_func.push_back(&grayscale_map);
		}
		else if(map=="normal")
		{
			feature_map_func.push_back(&normal_map);
		}
		else
		{
			return false;
		}
		
		struct transformation_state
		{
			long double alpha, x, y;
			transformation_state operator+(const transformation_state& rhs)
			{
				return transformation_state(this->alpha+rhs.alpha, this->x+rhs.x, this->y+rhs.y);
			}
			transformation_state& operator=(const transformation_state& rhs)
			{
				this->alpha=rhs.alpha; this->x=rhs.x; this->y=rhs.y;
				return *this;
			}
			transformation_state(long double nalpha, long double nx, long double ny)
			{
				this->alpha=nalpha; this->x=nx; this->y=ny;
			}
		};
		
		float dist_to_angle=0.05235987756, dist_to_prob_dist=5.0;	// fine tune dist_to_prob_dist
		int n_random_states=0;	// add later
		float step_ratio=2.0, cell_reduction_constant=2.0;
		
		float cell_size=3.0;
		float step_coeff=2.0;

		transformation_state best_transformation(0.0, 0.0, 0.0);
		while(cell_size>=min_cell_size/2.0)
		{
			float step=cell_size*step_coeff;
//			float smallest=cell_size/(2.0*dist_to_prob_dist);
			float smallest=cell_size/4.0;
			
			std::vector<point> r(reading);
			build_transform(transformation_mat, best_transformation.alpha, best_transformation.x, best_transformation.y);
			transform(r, transformation_mat);
			std::pair<pii, vvf> feature_map1, feature_map2;
			ld best_mi=0.0;
			for(int mit=0; mit<int(feature_map_func.size()); ++mit)
			{
				feature_map_func[mit](r, cell_size, feature_map1);
				feature_map_func[mit](reference, cell_size, feature_map2);
				long double temp=map_weightage_coeff[mit]*mutual_information(feature_map1, feature_map2, hist_size);
				best_mi+=temp;
			}
			while(step>smallest)
			{
				std::vector<transformation_state> states;
			
				// uniformly selected states
				states.push_back(transformation_state(step*dist_to_angle, 0.0, 0.0));
				states.push_back(transformation_state(-step*dist_to_angle, 0.0, 0.0));
				states.push_back(transformation_state(0.0, step, 0.0));
				states.push_back(transformation_state(0.0, -step, 0.0));
				states.push_back(transformation_state(0.0, 0.0, step));
				states.push_back(transformation_state(0.0, 0.0, -step));

				states.push_back(transformation_state(step*dist_to_angle, step, 0.0));
				states.push_back(transformation_state(step*dist_to_angle, -step, 0.0));
				states.push_back(transformation_state(-step*dist_to_angle, step, 0.0));
				states.push_back(transformation_state(-step*dist_to_angle, -step, 0.0));
				states.push_back(transformation_state(step*dist_to_angle, 0.0, step));
				states.push_back(transformation_state(step*dist_to_angle, 0.0, -step));
				states.push_back(transformation_state(-step*dist_to_angle, 0.0, step));
				states.push_back(transformation_state(-step*dist_to_angle, 0.0, -step));
				states.push_back(transformation_state(0.0, step, step));
				states.push_back(transformation_state(0.0, step, -step));
				states.push_back(transformation_state(0.0, -step, step));
				states.push_back(transformation_state(0.0, -step, -step));

				states.push_back(transformation_state(step*dist_to_angle, step, step));
				states.push_back(transformation_state(step*dist_to_angle, step, -step));
				states.push_back(transformation_state(step*dist_to_angle, -step, step));
				states.push_back(transformation_state(step*dist_to_angle, -step, -step));
				states.push_back(transformation_state(-step*dist_to_angle, step, step));
				states.push_back(transformation_state(-step*dist_to_angle, step, -step));
				states.push_back(transformation_state(-step*dist_to_angle, -step, step));
				states.push_back(transformation_state(-step*dist_to_angle, -step, -step));
								
				// randomly selected states
				std::random_device rd; // obtain a random number from hardware
				std::mt19937 eng(rd()); // seed the generator
				std::uniform_real_distribution<> distr(0.0, step*dist_to_prob_dist); // define the range
				for(int i=0; i<n_random_states; ++i) states.push_back(transformation_state(distr(eng)*dist_to_angle, distr(eng), distr(eng)));
			
				ld better_mi=best_mi;
				transformation_state better_transformation(0.0, 0.0, 0.0);
				int n_selection=-1;
				for(int i=0; i<int(states.size()); ++i)
				{
					r.clear();
					r=reading;
					build_transform(transformation_mat, best_transformation.alpha+states[i].alpha, best_transformation.x+states[i].x, best_transformation.y+states[i].y);
					transform(r, transformation_mat);
					ld mi=0.0;
					for(int mit=0; mit<int(feature_map_func.size()); ++mit)
					{
						feature_map_func[mit](r, cell_size, feature_map1);
						feature_map_func[mit](reference, cell_size, feature_map2);
						long double temp=map_weightage_coeff[mit]*mutual_information(feature_map1, feature_map2, hist_size);
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
						if(states[n_selection].x>step) step=states[n_selection].x;
					}
					step*=step_ratio;
				}
				else
				{
					step/=step_ratio;
				}
			}
			cell_size/=cell_reduction_constant;
		}
		build_transform(transformation_mat, best_transformation.alpha, best_transformation.x, best_transformation.y);
		transform(reading, transformation_mat);
		return true;
}
