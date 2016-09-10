#include <fstream>
#include <iostream>
#include <random>

using namespace std;

int main(int argc, char** argv)
{
	if(argc!=4)
	{
		std::cout<<"usage: ./random_input scan1 scan2 #inputs\n";
	}
	else
	{	
		string output_name;
		string f1(argv[1]), f2(argv[2]), f3(argv[3]);
		output_name.append(f1); output_name.push_back('_');
		output_name.append(f2); output_name.push_back('_');
		output_name.append(f3); output_name.push_back('_');
		output_name.append("input.txt");
		
		ofstream file(output_name);
		if(file.is_open())
		{
			random_device rd;
			mt19937 eng(rd());
			uniform_real_distribution<long double> distr1(-3.0, 3.0), distr2(-30.0, 30.0);
			int inputs=stoi(argv[3]);
			for(int i=0; i<inputs; ++i)
			{
				file<<distr1(eng)<<" "<<distr1(eng)<<" "<<distr1(eng)<<" "<<distr2(eng)<<" "<<distr2(eng)<<" "<<distr2(eng)<<endl;
			}
			return true;
		}
		else
		{
			return false;
		}		
	}
	return 0;
}
