#include "Route_VRP.h"
#include "../Graph/Graph_VRP.h"

route_VRP::route_VRP(graph_VRP* Gg){
	G = Gg;
};

void route_VRP::addNode(node_VRP* newnode){
	nodes.push_back(newnode);
};
float route_VRP::totalLength(){
	list<node_VRP*>::iterator it_next, it;
	float length = 0;
	//on itère sur 2 elements de la liste
	for( it = nodes.begin(), it_next = ++nodes.begin(); it != nodes.end(); it++, it_next++){
		//Le dernier noeud est relié au premier
		if(it_next == nodes.end()){
			it_next = nodes.begin();
		}
		length+= G->distance((*it)->num, (*(it_next))->num);
	}
	return length;
};

float route_VRP::totalDemand(){
	list<node_VRP*>::iterator it;
	float demand = 0;
	for(it = nodes.begin(); it != nodes.end(); it++){
		if((*it)->num != 0){
			demand += (*it)->demand;
		}
	}
	return demand;
}
