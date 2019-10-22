#include <ilcplex/ilocplex.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <sstream>

#include "Graph/Graph_VRP.h"
#include "Route/Route_VRP.h"
using namespace std;

int main(int argc, char**argv){

	if(argc!=2){
		cerr << "usage: "<< argv[0]<<" instance file name "<<endl;
	}

	int i, j;
	string filename = argv[1];

	ifstream fic(filename.c_str());

	if (!fic){
		cerr<<"file "<<filename<<" not found"<<endl;
		return 1;
	}

	graph_VRP G;

	G.read_file(fic);

	for(i = 0; i<G.nb_nodes; i++){
		cout << G.V_nodes[i].num << " : " << G.V_nodes[i].demand << endl;
	}
	for(i = 0; i<G.nb_links; i++){
		cout << G.V_links[i]->v1 << " " << G.V_links[i]->v2 << " : " << G.V_links[i]->length<< endl;
	}

	route_VRP R(&G);
	R.addNode(&G.V_nodes[0]);
	R.addNode(&G.V_nodes[1]);
	R.addNode(&G.V_nodes[2]);


	cout << "Longueur total du parcours: " <<R.totalLength() << " Demande total: " << R.totalDemand()<< endl;

	fic.close();
}
