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
		vector<pair<int, int>> scan_pairs;
		ofstream file("scan_pairs.txt");
		if(file.is_open())
		{
			while(scan_pairs.size()!=n)
			{
				random_device rd;
				mt19937 eng(rd());
				uniform_int_distribution<> distr1(1, 3789), distr2(1, 10);
				pair<int, int> scan_pair;
				scan_pair.second=distr1(eng);
				scan_pair.first=scan_pair.second+distr2(eng);
				if(scans.find(scan_pair.first)==scans.end()&&scans.find(scan_pair.second)==scans.end())
				{
					scan_pairs.push_back(scan_pair);
					scans.insert(scan_pair.first);
					scans.insert(scan_pair.second);
				}
			}
			for(auto it : scan_pairs)
			{
				file<<setfill('0')<<setw(4)<<it.first<<" "<<setfill('0')<<setw(4)<<it.second<<endl;
			}
		}
		else
		{
			cout<<"File failed to open! Exiting..."<<endl;
		}
	}
	else
	{
		cout<<"usage: ./test_generator #cases";
	}
	
}
