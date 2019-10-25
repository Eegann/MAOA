#include <ilcplex/ilocplex.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <sstream>
#include <vector>
#include <functional>
#include <algorithm>
#include "Graph/Graph_VRP.h"
#include "Route/Route_VRP.h"
#include "Algo/Greedy.cpp"
using namespace std;

int main(int argc, char**argv){

	if(argc!=4){
		cerr << "usage: "<< argv[0]<<"\n\t instance_file \n\t nb_vehicure \n\t vehicule_capacity  "<<endl;
	}

	int i, j, nb_vehicules, capacity;
	string filename = argv[1];

	nb_vehicules = atoi(argv[2]);
	capacity = atoi(argv[3]);

	ifstream fic(filename.c_str());

	if (!fic){
		cerr<<"file "<<filename<<" not found"<<endl;
		return 1;
	}

	graph_VRP G;

	G.read_file(fic);

	fic.close();

	cout << "End reading" << endl;

	vector<route_VRP> routes;

	greedyAlgorithm(&G, routes, nb_vehicules, capacity);

}
