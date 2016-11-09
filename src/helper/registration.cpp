#include <mireg.hpp>
#include <project_classes.hpp>
#include <pcl_visualizer.hpp>
#include <transformation.hpp>
#include <registration.hpp>

// TODO: Check whether to use double, float, or mixed for different purposes

void fill_sample_space(std::vector<Eigen::Vector3d>& states, double step, double dist_to_angle, double dist_to_prob_dist)
{
	states.clear(); states.reserve(27);

	Eigen::Vector3d temp;
	
	// uniformly selected 27 states
	temp=Eigen::Vector3d(0.0, 0.0, 0.0); states.push_back(temp);
	
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
}

// TODO: 1. Fix cache misses, and branching
// TODO: 2. Try multithreading across cpu check it on your computer and server try to bring down the time to less than 1 second
// TODO: 3. Port it to cuda
class information_tree
{
public:
	static const unsigned short int grid_size = 100;	// Keep it even TODO: Bring in half grid_size to handle this
	unsigned int grid[grid_size][grid_size];
	static const unsigned int max_unsigend_int = std::numeric_limits<unsigned int>::max();
	static constexpr float min_float = std::numeric_limits<float>::lowest();
	static constexpr float max_float = std::numeric_limits<float>::max();
	unsigned short int hist_size, max_level;
	float max_cell_size;
	float event_min_value, event_max_value;
	float tree_min_value, tree_max_value;
	struct node
	{
		unsigned int children[4];
		float value;
		node()
		{
			children[0] = max_unsigend_int; children[1] = max_unsigend_int; children[2] = max_unsigend_int; children[3] = max_unsigend_int;
			value = -1.0;
		}
	};
	std::vector<unsigned int> count;
	std::vector<float> sum;
	std::vector<node> tree;
	std::vector<unsigned int> events;

	void insert(float x, float y, float value)
	{
		unsigned short int gridx = (x + max_cell_size * (grid_size/2)) / max_cell_size;
		unsigned short int gridy = (y + max_cell_size * (grid_size/2)) / max_cell_size;

		float curr_coordinate[2];
		curr_coordinate[0] = ((float)gridx - (grid_size/2))*max_cell_size + max_cell_size/2;
		curr_coordinate[1] = ((float)gridy - (grid_size/2))*max_cell_size + max_cell_size/2;

		if(grid[gridx][gridy] >= tree.size())
		{
			grid[gridx][gridy] = tree.size();
			tree.push_back(node());
			count.push_back(0);
			sum.push_back(0);
		}

		unsigned int curr_node_index = grid[gridx][gridy];
		sum[curr_node_index] += value;
		count[curr_node_index]++;

		float curr_step_size = max_cell_size / 4;
		for(unsigned int curr_level = 1; curr_level < max_level; ++curr_level)
		{
			if(x >= curr_coordinate[0])
			{
				if(y >= curr_coordinate[1])
				{
					if(tree[curr_node_index].children[3] >= tree.size())
					{
						tree[curr_node_index].children[3] = tree.size();
					}
					curr_node_index = tree[curr_node_index].children[3];
					curr_coordinate[0] += curr_step_size;
					curr_coordinate[1] += curr_step_size;
				}
				else
				{
					if(tree[curr_node_index].children[2] >= tree.size())
					{
						tree[curr_node_index].children[2] = tree.size();
					}
					curr_node_index = tree[curr_node_index].children[2];
					curr_coordinate[0] += curr_step_size;
					curr_coordinate[1] -= curr_step_size;
				}
			}
			else
			{
				if(y >= curr_coordinate[1])
				{
					if(tree[curr_node_index].children[1] >= tree.size())
					{
						tree[curr_node_index].children[1] = tree.size();
					}
					curr_node_index = tree[curr_node_index].children[1];
					curr_coordinate[0] -= curr_step_size;
					curr_coordinate[1] += curr_step_size;
				}
				else
				{
					if(tree[curr_node_index].children[0] >= tree.size())
					{
						tree[curr_node_index].children[0] = tree.size();
					}
					curr_node_index = tree[curr_node_index].children[0];
					curr_coordinate[0] -= curr_step_size;
					curr_coordinate[1] -= curr_step_size;
				}
			}
			if(curr_node_index == tree.size())
			{
				tree.push_back(node());
				sum.push_back(0);
				count.push_back(0);
			}
			sum[curr_node_index] += value;
			count[curr_node_index]++;

			curr_step_size /= 2;
		}
	}

	void event_insert(float x, float y, float value, unsigned short int req_level)
	{
		unsigned short int gridx = (x + max_cell_size * (grid_size/2)) / max_cell_size;
		unsigned short int gridy = (y + max_cell_size * (grid_size/2)) / max_cell_size;

		float curr_coordinate[2];
		curr_coordinate[0] = ((float)gridx - (grid_size/2))*max_cell_size + max_cell_size/2;
		curr_coordinate[1] = ((float)gridy - (grid_size/2))*max_cell_size + max_cell_size/2;

		unsigned int curr_node_index = grid[gridx][gridy];
		float curr_step_size = max_cell_size / 4;
		for(unsigned short int curr_level = 1; curr_level <= req_level; ++curr_level)
		{
			if(curr_node_index >= tree.size()) break;
			
			if(x >= curr_coordinate[0])
			{
				if(y >= curr_coordinate[1])
				{
					curr_node_index = tree[curr_node_index].children[3];
					curr_coordinate[0] += curr_step_size;
					curr_coordinate[1] += curr_step_size;
				}
				else
				{
					curr_node_index = tree[curr_node_index].children[2];
					curr_coordinate[0] += curr_step_size;
					curr_coordinate[1] -= curr_step_size;
				}
			}
			else
			{
				if(y >= curr_coordinate[1])
				{
					curr_node_index = tree[curr_node_index].children[1];
					curr_coordinate[0] -= curr_step_size;
					curr_coordinate[1] += curr_step_size;
				}
				else
				{
					curr_node_index = tree[curr_node_index].children[0];
					curr_coordinate[0] -= curr_step_size;
					curr_coordinate[1] -= curr_step_size;
				}
			}
			
			curr_step_size /= 2;
		}

		if(curr_node_index < tree.size())
		{
			if(count[curr_node_index] == 0) events.push_back(curr_node_index);
			count[curr_node_index]++;
			sum[curr_node_index] += value;
		}
	}

	void push_tree_values()
	{
		for(unsigned int i=0; i<tree.size(); ++i)
		{
			float value = sum[i] / count[i];
			tree[i].value = value;
			sum[i] = 0;
			count[i] = 0;
		}
	}
	
	float extract_information()
	{
		float mini = std::min(tree_min_value, event_min_value);
		float maxi = std::max(tree_max_value, event_max_value);
		float bin_size = (maxi - mini)/(hist_size - 1);

		std::vector<float> hist1(hist_size, 0.0), hist2(hist_size, 0.0);
		std::vector<std::vector<float>> hist_joint(hist_size, std::vector<float>(hist_size, 0.0));

		for(std::vector<unsigned int>::iterator it = events.begin(); it != events.end(); ++it)
		{
			unsigned int node_index = *it;

			float v1 = sum[node_index] / count[node_index];
			sum[node_index] = 0.0;
			count[node_index] = 0;
			unsigned short int index1 = (unsigned short int)((v1 - mini)/bin_size);
			hist1[index1] += 1.0;

			float v2 = tree[node_index].value;
			unsigned short int index2 = (unsigned short int)((v2 - mini)/bin_size);
			hist2[index2] += 1.0;

			hist_joint[index1][index2] += 1.0;
		}

		unsigned short int hist_count = events.size();		
		for(unsigned short int i=0; i < hist_size; ++i)
		{
			hist1[i] /= hist_count;	// TODO: Make assertions for hist_count == 0
			hist2[i] /= hist_count;	// TODO: Make assertions for hist_count == 0
		}

		events.clear();

		float mi = 0;
		for(unsigned short int i=0; i < hist_size; ++i)
		{
			for(unsigned short int j=0; j < hist_size; ++j)
			{
				if(hist_joint[i][j] != 0)
				{
					mi += hist_joint[i][j] * std::log2l(hist_joint[i][j]/(hist1[i]*hist2[j]));
				}
			}
		}

		return mi;
	}
	
public:
	void build_information_tree(std::vector<information_node>& cloud_nodes)
	{
		std::vector<information_node> node_bins[grid_size][grid_size];
		for(auto it = cloud_nodes.begin(); it != cloud_nodes.end(); ++it)
		{
			unsigned int gridx = (it->x + max_cell_size * (grid_size/2)) / max_cell_size;
			unsigned int gridy = (it->y + max_cell_size * (grid_size/2)) / max_cell_size;
			node_bins[gridx][gridy].push_back(*it);
		}

		for(int i=0; i<grid_size; ++i)
		{
			for(int j=0; j<grid_size; ++j)
			{
				for(auto it = node_bins[i][j].begin(); it != node_bins[i][j].end(); ++it)
				{
					insert(it->x, it->y, it->v);	// TODO: Reason why it's not giving speed gains as expected
				}
			}
		}
		
		push_tree_values();
	}
	
	void build_and_extract_information(std::vector<information_node>& cloud_nodes, Eigen::Vector3d transformation_state, unsigned short int req_level, float& mi)
	{
		double a = cos(transformation_state(2)), b = sin(transformation_state(2));	// 2D transformation coeffecients
		double e = transformation_state(0), f = transformation_state(1);

		// IDEA: Can try to build adaptive insert function which chooses point based on locality
		for(std::vector<information_node>::iterator it = cloud_nodes.begin(); it != cloud_nodes.end(); ++it)
		{
			float new_x = a * it->x - b * it->y + e;
			float new_y = b * it->x + a * it->y + f;
			event_insert(new_x, new_y, it->v, req_level);
		}
		mi = extract_information();	// TODO: Put a check(i.e. assert) level less than max_level
	}
	
	information_tree(float _max_cell_size, unsigned short int _hist_size, std::pair<float, float> event_minmax, std::pair<float, float> tree_minmax, unsigned int _max_level = 9) : max_cell_size(_max_cell_size), hist_size(_hist_size), max_level(_max_level)
	{
		event_min_value = event_minmax.first;
		event_max_value = event_minmax.second;
		tree_min_value = tree_minmax.first;
		tree_max_value = tree_minmax.second;
		for(unsigned short int i=0; i<grid_size; ++i)
		{
			for(unsigned short int j=0; j<grid_size; ++j)
			{
				grid[i][j] = max_unsigend_int;
			}
		}
	}
};

std::pair<float, float> process_points_variance(std::vector<point>& cloud, std::vector<information_node>& cloud_nodes)
{
	std::pair<float, float> ret(std::numeric_limits<float>::max(), std::numeric_limits<float>::lowest());
	
	unsigned int size = cloud.size();
	cloud_nodes = std::vector<information_node>(size);
	for(int i=0; i<size; ++i)
	{
		cloud_nodes[i].x = cloud[i].coordinate(0);
		cloud_nodes[i].y = cloud[i].coordinate(1);
		cloud_nodes[i].v = cloud[i].coordinate(2) * cloud[i].coordinate(2);
		
		ret.first = std::min(cloud_nodes[i].v, ret.first);
		ret.second = std::max(cloud_nodes[i].v, ret.second);
	}
	return ret;
}

std::pair<float, float> process_points_reflectivity(std::vector<point>& cloud, std::vector<information_node>& cloud_nodes)
{
	std::pair<float, float> ret(std::numeric_limits<float>::max(), std::numeric_limits<float>::lowest());
	
	unsigned int size = cloud.size();
	cloud_nodes = std::vector<information_node>(size);
	for(int i=0; i<size; ++i)
	{
		cloud_nodes[i].x = cloud[i].coordinate(0);
		cloud_nodes[i].y = cloud[i].coordinate(1);
		cloud_nodes[i].v = cloud[i].ref * cloud[i].ref;
		
		ret.first = std::min(cloud_nodes[i].v, ret.first);
		ret.second = std::max(cloud_nodes[i].v, ret.second);
	}
	return ret;
}

std::pair<float, float> process_points_grayscale(std::vector<point>& cloud, std::vector<information_node>& cloud_nodes)
{
	std::pair<float, float> ret(std::numeric_limits<float>::max(), std::numeric_limits<float>::lowest());
	
	unsigned int size = cloud.size();
	cloud_nodes = std::vector<information_node>(size);
	for(int i=0; i<size; ++i)
	{
		cloud_nodes[i].x = cloud[i].coordinate(0);
		cloud_nodes[i].y = cloud[i].coordinate(1);
		cloud_nodes[i].v = cloud[i].gray * cloud[i].gray;
		
		ret.first = std::min(cloud_nodes[i].v, ret.first);
		ret.second = std::max(cloud_nodes[i].v, ret.second);
	}
	return ret;
}

bool multires_registration(std::vector<point>& reading, std::vector<point>& reference, Eigen::Matrix4d& transformation_mat, double min_cell_size, int hist_size, std::string& map)
{
		std::vector<information_node> reading_nodes, reference_nodes;
		std::pair<float, float> event_minmax, tree_minmax;
		if(map=="variance")
		{
			event_minmax = process_points_variance(reading, reading_nodes);
			tree_minmax = process_points_variance(reference, reference_nodes);
		}
		else if(map=="reflectivity")
		{
			event_minmax = process_points_reflectivity(reading, reading_nodes);
			tree_minmax = process_points_reflectivity(reference, reference_nodes);
		}
		else if(map=="grayscale")
		{
			event_minmax = process_points_grayscale(reading, reading_nodes);
			tree_minmax = process_points_grayscale(reference, reference_nodes);		
		}
		else if(map=="normal")
		{
			std::cout<<"normal will be supported soon!!"<<std::endl;	// TODO: Get it live
			exit(0);
		}
		else if(map=="all")
		{
			std::cout<<"all will be supported soon!!"<<std::endl;	// TODO: Get it live
			exit(0);
		}
		else
		{
			std::cout<<"Usage: ./mireg_serial reading_file reference_file {variance, reflectivity, grayscale, normal, all}"<<std::endl;
			exit(0);
		}
		double max_cell_size=3.0;

		information_tree ref_tree(max_cell_size, hist_size, event_minmax, tree_minmax, 10);
		ref_tree.build_information_tree(reference_nodes);
		
		double dist_to_angle=0.05235987756, dist_to_prob_dist=5.0;	// fine tune dist_to_prob_dist
		double step_ratio=2.0, cell_reduction_constant=2.0;
		
		double cell_size = max_cell_size;
		double step_coeff=2.0;

		Eigen::Vector3d best_transformation(0.0, 0.0, 0.0);
		for(unsigned int curr_level = 0; curr_level < 10; ++curr_level)
		{
			double step=cell_size*step_coeff;
			double smallest=cell_size/4.0;
			
			float best_mi;
			while(step>smallest)
			{
				std::vector<Eigen::Vector3d> states;
				fill_sample_space(states, step, dist_to_angle, dist_to_prob_dist);
				
				std::vector<float> mis(states.size());
				for(int i = 0; i < states.size(); ++i)
				{
					ref_tree.build_and_extract_information(reading_nodes, best_transformation + states[i], curr_level, mis[i]);
				}
				
				int n_selection = 0;
				best_mi = mis[0];	// TODO: Assert if mis is empty
				for(int i = 1; i < mis.size(); ++i)
				{
					if(mis[i] > best_mi)
					{
						n_selection = i;
						best_mi = mis[i];
					}
				}
				if(n_selection != 0)
				{
					best_transformation=best_transformation + states[n_selection];
					step*=step_ratio;
				}
				else
				{
					step/=step_ratio;
				}
			}
			std::cout<<"cell_size: "<<cell_size<<", cells_best_mi: "<<best_mi<<std::endl;	// TODO: Remove at end
			cell_size/=cell_reduction_constant;
		}
		build_transform_z(transformation_mat, best_transformation);
		transform(reading, transformation_mat);
		return true;
}
