#ifndef GRAPH_VRP
#define GRAPH_VRP

#include <lemon/list_graph.h>
#include <vector>
#include <map>

using namespace std;

class link_VRP{
public:
	int num;
	int v1, v2;
	float length;
	int return_other_extrem(int v);
	lemon::ListGraph::Edge LGU_name;
};

class node_VRP{
public:
	int num;
	float demand;
	float x, y;

	list<link_VRP*> L_links;

	bool test_neighbour(int j);

	lemon::ListGraph::Node LGU_name;
	lemon::ListDigraph::Node LGD_name;
};

class graph_VRP{
public:
	int nb_nodes;
	int nb_links;
	// Encoding of the graph by adjacence list, i.e. a vector of list of edges 
	vector <node_VRP> V_nodes;

	// Additional encoding: a vector on the edges (on pointers over edges)
	vector <link_VRP*> V_links;

	bool directed;

	/*********************************************/
	/********* LEMON Structure *******************/
	lemon::ListGraph L_GU;
	lemon::ListDigraph L_GD;
	map<int, int> L_rtnmap; // map between the num of the ad hoc graph
                             // and the lemon id of a node
	void construct_Undirected_Lemon_Graph();
	void construct_Directed_Lemon_Graph();
	void read_file(istream & fic);
	float distance(int i, int j);

};
#endif
