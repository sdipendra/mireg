#include <mutual_information.hpp>


double mutual_information(std::vector<std::pair<pii, double>>& feature_map1, std::vector<std::pair<pii, double>>& feature_map2, int hist_size)
{
	double maxi = -std::numeric_limits<double>::max(), mini=std::numeric_limits<double>::max();
	for(std::vector<std::pair<pii, double>>::iterator it = feature_map1.begin(); it != feature_map1.end(); ++it)
	{
		maxi = std::max(maxi, it->second);
		mini = std::min(mini, it->second);
	}
	for(std::vector<std::pair<pii, double>>::iterator it = feature_map2.begin(); it != feature_map2.end(); ++it)
	{
		maxi = std::max(maxi, it->second);
		mini = std::min(mini, it->second);
	}

	double bin_size=(maxi-mini)/(hist_size-1);	// TODO: check if possible to replace with hist_size without any segmentation fault
	
	vd hist1(hist_size, 0.0), hist2(hist_size, 0.0);
	vvd hist_joint(hist_size, vd(hist_size, 0.0));

	int hist1_count=0, hist2_count=0, hist_joint_count=0;

	int size1=feature_map1.size(), size2=feature_map2.size();
	int i=0, j=0;
	while(i<size1||j<size2)
	{
		if(j==size2 || (i!=size1 && feature_map1[i].first<feature_map2[j].first))
		{
			hist1[int((feature_map1[i].second-mini)/bin_size)]+=1.0;
			++hist1_count;
			++i;
		}
		else if(i==size1 || (j!=size2 && feature_map1[i].first>feature_map2[j].first))
		{
			hist2[int((feature_map2[j].second-mini)/bin_size)]+=1.0;
			++hist2_count;
			++j;
		}
		else
		{
			hist_joint[int((feature_map1[i].second-mini)/bin_size)][int((feature_map2[j].second-mini)/bin_size)]+=1.0;	
			++hist_joint_count;

			hist1[int((feature_map1[i].second-mini)/bin_size)]+=1.0;
			++hist1_count;
			++i;
			
			hist2[int((feature_map2[j].second-mini)/bin_size)]+=1.0;
			++hist2_count;
			++j;
		}
	}
	for(int i=0; i<hist_size; ++i) hist1[i]/=hist1_count;	// normalization
	for(int i=0; i<hist_size; ++i) hist2[i]/=hist2_count;	// normalization
//	for(int i=0; i<hist_size; ++i) for(int j=0; j<hist_size; ++j) hist_joint[i][j]/=hist_joint_count;	// I was not doing it earlier OOPS!!!
//	TODO: THE ABOVE LINE SHOULD BE THERE AS FAR AS I UNDERSTAND BUT IT IS CREATING PROBLEMS CHECK THIS
	
	double mi=0.0;
	for(int i=0; i<hist_size; ++i)
	{
		for(int j=0; j<hist_size; ++j)
		{
			if(hist_joint[i][j]!=0.0)	// TODO: CHECK IF THESE VALUES ARE TOO MUCH THAN USE SOME OTHER DATA STRUCTURE TO STORE VALUES SMARTLY
			{
				mi+=hist_joint[i][j]*std::log2l(hist_joint[i][j]/(hist1[i]*hist2[j]));	// if hist_joint[i][j] is zero then both of them has to be zero
			}
		}
	}
	return mi;
}
