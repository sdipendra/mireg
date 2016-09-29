#ifndef _MIREG_H_	// to include the header only once
#define _MIREG_H_

// takes care of outside dependency and custom classes and structs of the program throughout the program
#include <algorithm>
#include <fstream>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>

#include <thrust/reduce.h>
#include <thrust/device_vector.h>

#include <gsl/gsl_blas.h>
#include <gsl/gsl_eigen.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_histogram.h>
#include <gsl/gsl_histogram2d.h>
#include <gsl/gsl_test.h>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_siman.h>
#include <gsl/gsl_ieee_utils.h>


// Short hands for data types being used in the program
typedef long double ld;
typedef std::vector<int> vi;
typedef std::vector<vi> vvi;
typedef std::vector<float> vf;
typedef std::vector<vf> vvf;
typedef std::vector<vvf> vvvf;
//typedef std::vector<char> vc;
//typedef std::vector<vc> vvc;
//typedef std::vector<string> vs;
typedef std::pair<int, int> pii;
typedef std::vector<pii> vpii;
typedef std::vector<vpii> vvpii;
//typedef std::vector<bool> vb;
//typedef std::vector<vb> vvb;
//typedef std::priority_queue<int, std::vector<int>, greater<int>> min_heap_int;
//typedef std::priority_queue<int, std::vector<int>, less<int>> max_heap_int;

//#define pi 3.141592653589793238462643383279502884197169399375105820974944592307816406286	// Why the error don't understand come back to it
const long double pi = 3.141592653589793238462643383279502884197169399375105820974944592307816406286;
//#define f(i, n) for(int i=0; i<n; ++i)
//#define fi(i, n, j) for(int i=j; i<n; ++i)
#define mp make_pair
//#define all(container) container.begin(), container.end()
//#define allr(container) container.rbegin(), container.rend()
//#define tr(container, it) for (decltype(container.begin()) it = container.begin(); it != container.end(); it++)
//#define trr(container, it) for (decltype(container.rbegin()) it = container.rbegin(); it != container.rend(); it++)
//#define dist(a, b) sqrtl(powl(ld(a.first - b.first), 2) + powl(ld(a.second - b.second), 2))


#endif
