#include "../Graph/Graph_VRP.h"
#include "../Route/Route_VRP.h"

bool sort_link(const link_VRP* link1, const link_VRP* link2){
        return link1->length < link2->length;
}


void greedyAlgorithm(graph_VRP* G, int nb_vehicules, int capacity){


	int i=0, solLength=0;
        //List of sorted nodes by distance from 0

        list<link_VRP*> sortedLinks;

        for(i = 0; i < G->nb_links; i++){
                if(G->V_links[i]->v1 == 0 || G->V_links[i]->v2 == 0){
                        sortedLinks.push_back(G->V_links[i]);
                }
        }

        sortedLinks.sort(sort_link);

        //Routes for vehicules

        vector<route_VRP> routes;

        for( i=0; i<nb_vehicules; i++){
                route_VRP R(G);
                R.addNode(&G->V_nodes[0]);
                routes.push_back(R);
        }

        for(list<link_VRP*>::iterator it=sortedLinks.begin(); it != sortedLinks.end(); it++){
                int bestDist = 50000;
                int best = -1;
                for( i = 0; i<nb_vehicules; i++){
                        if(routes.at(i).totalDemand() + G->V_nodes[(*it)->return_other_extrem(0)].demand <= capacity){
                                int tempDist = G->distance((*it)->return_other_extrem(0),(*(--routes.at(0).nodes.end()))->num);
                                if( tempDist < bestDist){
                                        bestDist = tempDist;
                                        best = i;
                                }
                        }
                }
                routes.at(best).addNode(&G->V_nodes[(*it)->return_other_extrem(0)]);
        }

	//Print results

        for( i = 0; i<routes.size(); i++){
                cout << "Route " << i+1 << " : ";
		for(list<node_VRP*>::iterator it=routes.at(i).nodes.begin(); it != routes.at(i).nodes.end(); it++){
                        cout << (*it)->num << " " ;
                }
                cout << endl;
                cout << "Length: " << routes.at(i).totalLength() << " Demand: " << routes.at(i).totalDemand() << endl;
		solLength+=routes.at(i).totalLength();
	}
	cout << "Longueur total: " << solLength << endl;
}

void greedyAlgorithm2(graph_VRP* G, int nb_vehicules, int capacity){

	int i=0, solLength=0;
	//List of sorted nodes by distance from 0

	list<link_VRP*> sortedLinks;

	for(i = 0; i < G->nb_links; i++){
		if(G->V_links[i]->v1 == 0 || G->V_links[i]->v2 == 0){
			 sortedLinks.push_back(G->V_links[i]);
		}
	}

	for(list<link_VRP*>::iterator it=sortedLinks.begin(); it != sortedLinks.end(); it++){
		(*it)->length=(*it)->length/G->V_nodes[(*it)->return_other_extrem(0)].demand;
	}

        sortedLinks.sort(sort_link);

        //Routes for vehicules

        vector<route_VRP> routes;

        for( i=0; i<nb_vehicules; i++){
                route_VRP R(G);
                R.addNode(&G->V_nodes[0]);
                routes.push_back(R);
        }

        for(list<link_VRP*>::iterator it=sortedLinks.begin(); it != sortedLinks.end(); it++){
                int bestDist = 50000;
                int best = -1;
                for( i = 0; i<nb_vehicules; i++){
                        if(routes.at(i).totalDemand() + G->V_nodes[(*it)->return_other_extrem(0)].demand <= capacity){
                                int tempDist = G->distance((*it)->return_other_extrem(0),(*(--routes.at(0).nodes.end()))->num) 
						+ G->distance(0, (*it)->return_other_extrem(0))
						- G->distance(0, (*(--routes.at(0).nodes.end()))->num);
				if( tempDist < bestDist){
                                        bestDist = tempDist;
                                        best = i;
                                }
                        }
                }
                routes.at(best).addNode(&G->V_nodes[(*it)->return_other_extrem(0)]);
        }


        //Print results

        for( i = 0; i<routes.size(); i++){
                cout << "Route " << i+1 << " : ";
                for(list<node_VRP*>::iterator it=routes.at(i).nodes.begin(); it != routes.at(i).nodes.end(); it++){
                        cout << (*it)->num << " " ;
                }
                cout << endl;
                cout << "Length: " << routes.at(i).totalLength() << " Demand: " << routes.at(i).totalDemand() << endl;
                solLength+=routes.at(i).totalLength();
        }
        cout << "Longueur total: " << solLength << endl;

}
