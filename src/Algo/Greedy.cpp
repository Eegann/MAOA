#include "../Graph/Graph_VRP.h"
#include "../Route/Route_VRP.h"

bool sort_link(const link_VRP* link1, const link_VRP* link2){
        return link1->length < link2->length;
}


void greedyAlgorithm(graph_VRP* G, vector<route_VRP>* routes, int nb_vehicules, int capacity){


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

        for( i=0; i<nb_vehicules; i++){
                route_VRP R(G);
                R.addNode(&G->V_nodes[0]);
                routes->push_back(R);
        }

        for(list<link_VRP*>::iterator it=sortedLinks.begin(); it != sortedLinks.end(); it++){
                int bestDist = 50000;
                int best = -1;
                for( i = 0; i<nb_vehicules; i++){
                        if(routes->at(i).totalDemand() + G->V_nodes[(*it)->return_other_extrem(0)].demand <= capacity){
                                int tempDist = G->distance((*it)->return_other_extrem(0),(*(--routes->at(i).nodes.end()))->num);
                                if( tempDist < bestDist){
                                        bestDist = tempDist;
                                        best = i;
                                }
                        }
                }
                routes->at(best).addNode(&G->V_nodes[(*it)->return_other_extrem(0)]);
        }


}
