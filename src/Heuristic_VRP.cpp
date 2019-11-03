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
#include "Algo/LocalSearch.cpp"
using namespace std;


void printResult(vector<route_VRP>* routes){

	int i;
	float solLength = 0;
	for( i = 0; i< routes->size(); i++){
		cout << "Route " << i+1 << " : ";
		for(list<node_VRP*>::iterator it=routes->at(i).nodes.begin(); it != routes->at(i).nodes.end(); it++){
			cout << (*it)->num << " " ;
		}
		cout << endl;
		cout << "Length: " << routes->at(i).totalLength() << " Demand: " << routes->at(i).totalDemand() << endl;
		solLength+=routes->at(i).totalLength();
	}
	cout << "Longueur total: " << solLength << endl;
}

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

	cout << "\n\tEnd reading\n" << endl;

	vector<route_VRP> routes;

	cout << "\n\tBegin greedy algorithm\n" << endl;

	greedyAlgorithm(&G, &routes, nb_vehicules, capacity);
	printResult(&routes);

	cout << "\n\tBegin local search\n" << endl;

	localSearch(&G, &routes, nb_vehicules, capacity);
	printResult(&routes);
}
