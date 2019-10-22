#ifndef ROUTE_VRP
#define ROUTE_VRP

#include "../Graph/Graph_VRP.h"
using namespace std;

class route_VRP{
public:
	graph_VRP* G;
	list<node_VRP*> nodes;
	route_VRP(graph_VRP* Gg);

	void addNode(node_VRP* newnode);
	float totalLength();
	float totalDemand();
};


#endif
