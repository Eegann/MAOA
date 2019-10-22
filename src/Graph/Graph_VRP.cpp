#include "Graph_VRP.h"
#include<cstring>
#include <lemon/lgf_writer.h>
#include <math.h>

#define GRAPHVIZ "$PATHTUTOMIP/graphviz-2.40.1/bin/"

using namespace std;

/****************************  link_VRP  *******************************/
int link_VRP::return_other_extrem(int v){
	return (v==v1?v2:v1);
}

/****************************  node_VRP  *******************************/
bool node_VRP::test_neighbour(int j){
  list<link_VRP*>::iterator it;
  for(it=L_links.begin() ; it !=L_links.end() ; it++){
    if((*it)->return_other_extrem(num) == j)
      return true;
  }
  return false;
}

/****************************  graph_VRP  *******************************/

void graph_VRP::read_file(istream & fic){
	int i,j,k;
	//Tab of char to read each line of the file
	char ch[100];
	link_VRP *link;

	//Read the file until we read "DIMENSION"
	fic>>ch;
	while ((strcmp(ch,"DIMENSION"))){
		fic>>ch;
	}
	//Stock the value in nb_nodes
	fic>>ch;
	if ((strcmp(ch,":"))) {
		nb_nodes=atoi(ch);
	}
	else{
		fic>>nb_nodes;
	}

	cout<<nb_nodes<<endl;

	V_nodes.resize(nb_nodes);

	//Read the file until "NODE_COORD_SECTION"
	fic>>ch;
	while ((strcmp(ch,"NODE_COORD_SECTION")))
		fic>>ch;

	//For each node, add it's value, x position and y position
	for (i=1;i<nb_nodes;i++){
		fic>>V_nodes[i].num;
		fic>>V_nodes[i].x;
		fic>>V_nodes[i].y;
	}

	//Read the file until "DEMAND_SECTION"
	while ((strcmp(ch,"DEMAND_SECTION")))
		fic>>ch;
	//For each node, add it's demand value

	int temp;
	for (i=0;i<nb_nodes;i++){
		fic>>temp;
		fic>>V_nodes[i].demand;
	}
	nb_links = (nb_nodes*(nb_nodes-1))/2;
	V_links.resize(nb_links);
	cout << V_links.size() << endl;
	k=0;
	for (i=0;i<nb_nodes;i++){
		for (j=i+1;j<nb_nodes;j++){
			link=new link_VRP;
			link->num=k;
			link->v1=i;
			link->v2=j;
			link->length=distance(i,j);
			V_nodes[i].L_links.push_back(link);
			V_nodes[j].L_links.push_back(link);
			V_links[k] = link;
			k++;
		}
	}
	directed=false;
	construct_Undirected_Lemon_Graph();

}

float graph_VRP::distance(int i, int j){
	//Pythagor formula
	return sqrt(pow(V_nodes[i].x-V_nodes[j].x,2)+pow(V_nodes[i].y-V_nodes[j].y,2));
}

/******************* LEMON ******************/

void graph_VRP::construct_Undirected_Lemon_Graph(){

  int i;
  list<link_VRP *>::const_iterator it;

  for (i=0;i<nb_nodes;i++){
    V_nodes[i].LGU_name=L_GU.addNode();
    L_rtnmap[L_GU.id(V_nodes[i].LGU_name)]=i;
  }
  for (i=0;i<nb_nodes;i++){
    for (it=V_nodes[i].L_links.begin();it!=V_nodes[i].L_links.end();it++){
	(*it)->LGU_name=L_GU.addEdge(V_nodes[i].LGU_name,V_nodes[(*it)->return_other_extrem(i)].LGU_name);

    }
  }

}
