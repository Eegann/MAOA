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
using namespace std;

bool sort_link(const link_VRP* link1, const link_VRP* link2){
	return link1->length < link2->length;
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

	for(i = 0; i<G.nb_nodes; i++){
		cout << G.V_nodes[i].num << " : " << G.V_nodes[i].demand << endl;
	}
	for(i = 0; i<G.nb_links; i++){
		cout << G.V_links[i]->v1 << " " << G.V_links[i]->v2 << " : " << G.V_links[i]->length<< endl;
	}

	//List of sorted nodes by distance from 0

	list<link_VRP*> sortedLinks;

	for(i = 0; i<G.nb_links; i++){
		if(G.V_links[i]->v1 == 0 || G.V_links[i]->v2 == 0){
			sortedLinks.push_back(G.V_links[i]);
		}
	}


//	sort(sortedNodes.begin(),sortedNodes.end(),bind(sort_node,placeholders::_1,placeholders::_2,G));

	sortedLinks.sort(sort_link);

	//Routes for vehicules

	vector<route_VRP> routes;

	for( i=0; i<nb_vehicules; i++){
		route_VRP R(&G);
		R.addNode(&G.V_nodes[0]);
		routes.push_back(R);
	}

	for(list<link_VRP*>::iterator it=sortedLinks.begin(); it != sortedLinks.end(); it++){
		int bestDist = 5000;
		int best = -1;
		for( i = 0; i<nb_vehicules; i++){
			if(routes.at(i).totalDemand() + G.V_nodes[(*it)->return_other_extrem(0)].demand <= capacity){
				int tempDist =G.distance((*it)->return_other_extrem(0),(*(--routes.at(0).nodes.end()))->num);
				if( tempDist < bestDist){
					bestDist = tempDist;
					best = i;
				}
			}
		}
		routes.at(best).addNode(&G.V_nodes[(*it)->return_other_extrem(0)]);
	}


	//Print things

	for( i = 0; i<routes.size(); i++){
		for(list<node_VRP*>::iterator it=routes.at(i).nodes.begin(); it != routes.at(i).nodes.end(); it++){
			cout << (*it)->num << " " ;
		}
		cout << endl;
		cout << "Length: " << routes.at(i).totalLength() << " Demand: " << routes.at(i).totalDemand() << endl;
	}


//	cout << "Longueur total du parcours: " <<R.totalLength() << " Demande total: " << R.totalDemand()<< endl;

	fic.close();
}
