#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <set>

using namespace std;

int main(int argc, char** argv)
{
	if(argc==2)
	{
		int n=stoi(argv[1]);
		set<int> scans;
		vector<int> nscan;
		ofstream file("reference_scans_exp2.txt");
		if(file.is_open())
		{
			random_device rd;
			mt19937 eng(rd());
			uniform_int_distribution<> distr1(0, 125), distr2(1, 15);
			while(nscan.size()!=n)
			{
				int scan_base=distr1(eng);
				int scan_num=30*scan_base+distr2(eng);
				if(scans.find(scan_base)==scans.end())
				{
					nscan.push_back(scan_num);
					scans.insert(scan_base);
				}
			}
			for(auto it : nscan)
			{
				file<<it<<endl;
			}
		}
		else
		{
			cout<<"File failed to open! Exiting..."<<endl;
		}
	}
	else
	{
		cout<<"usage: ./test_generator_exp2 #cases"<<endl;
	}
	
}
