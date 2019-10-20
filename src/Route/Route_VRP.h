#ifndef ROUTE_VRP
#define ROUTE_VRP

#include "../Graph/Graph_VRP.h"
using namespace std;

class route_VRP{
	list<node_VRP*> nodes;

	void addNode(node_VRP* newnode);
};


#endif
