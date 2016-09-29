#include <mutual_information.hpp>

const int low_filter = 1;	// Reject overly populated low values adjust the parameter

ld mutual_information(std::pair<pii, vvf>& feature_map1,std::pair<pii, vvf>& feature_map2, int hist_size)
{
	int rows1 = feature_map1.second.size();
	int cols1 = feature_map1.second[0].size();
	float maxi = -std::numeric_limits<float>::max(), mini=std::numeric_limits<float>::max();
	for(int i=0; i<rows1; ++i)
	{
		for(int j=0; j<cols1; ++j)
		{
			if(feature_map1.second[i][j]>maxi) maxi=feature_map1.second[i][j];
			if(feature_map1.second[i][j]<mini) mini=feature_map1.second[i][j];
		}
	}
	
	int rows2 = feature_map2.second.size();
	int cols2 = feature_map2.second[0].size();
	for(int i=0; i<rows2; ++i)
	{
		for(int j=0; j<cols2; ++j)
		{
			if(feature_map2.second[i][j]>maxi) maxi=feature_map2.second[i][j];
			if(feature_map2.second[i][j]<mini) mini=feature_map2.second[i][j];
		}
	}
	
	float bin_size=(maxi-mini)/(hist_size-1);
	
	vf hist1(hist_size, 0.0);
	int hist1_count=0;
	for(int i=0; i<rows1; ++i)
	{
		for(int j=0; j<cols1; ++j)
		{
			if((feature_map1.second[i][j]-mini-low_filter*bin_size)/bin_size>0.0)
			{
				hist1[int((feature_map1.second[i][j]-mini-low_filter*bin_size)/bin_size)]+=1.0;
				hist1_count++;
			}
		}
	}
	for(int i=0; i<hist_size; ++i) hist1[i]/=float(hist1_count);	// normalization
	
	vf hist2(hist_size, 0.0);
	int hist2_count=0;
	for(int i=0; i<rows2; ++i)
	{
		for(int j=0; j<cols2; ++j)
		{
			if((feature_map2.second[i][j]-mini-low_filter*bin_size)/bin_size>0.0)
			{
				hist2[int((feature_map2.second[i][j]-mini-low_filter*bin_size)/bin_size)]+=1.0;
				hist2_count++;
			}
		}
	}
	for(int i=0; i<hist_size; ++i) hist2[i]/=float(hist2_count);	// normalization
	
	vvf hist_joint(hist_size, vf(hist_size, 0.0));
	int hist_joint_count=0;

	for(int i1=0; i1<rows1; ++i1)
	{
		for(int j1=0; j1<cols1; ++j1)
		{
			if((feature_map1.second[i1][j1]-mini-low_filter*bin_size)/bin_size>0.0)
			{
				int i2=i1-feature_map1.first.first+feature_map2.first.first, j2=j1-feature_map1.first.second+feature_map2.first.second;
				if(i2>0&&i2<feature_map2.second.size()&&j2>0&&j2<feature_map2.second[0].size())
				{
					if((feature_map2.second[i2][j2]-mini-low_filter*bin_size)/bin_size>0.0)
					{
						hist_joint[int((feature_map1.second[i1][j1]-mini-low_filter*bin_size)/bin_size)][int((feature_map2.second[i2][j2]-mini-low_filter*bin_size)/bin_size)]+=1.0;
						hist_joint_count++;
					}
				}
			}
		}
	}
	
	ld mi=0.0;
	for(int i=0; i<hist_size; ++i)
	{
		for(int j=0; j<hist_size; ++j)
		{
			if(hist_joint[i][j]!=0.0)
			{
				mi+=hist_joint[i][j]*std::log2l(hist_joint[i][j]/(hist1[i]*hist2[j]));	// if hist_joint[i][j] is zero then both of them has to be zero
			}
		}
	}
	return mi;
}
