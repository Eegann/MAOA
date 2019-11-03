void localSearch(graph_VRP* G, vector<route_VRP>* routes, int nb_vehicules, int capacity){
	srand(time(NULL));

	int nb_iter = 0;

	while(nb_iter < 10000){
		nb_iter++;

		//randr = random route, randn = random node
		int i, randr1, randr2, randn1, randn2;
		vector<int> possibleRoute;

		randr1 = rand() % routes->size();
		//+1 car on ne peut pas bouger le premier noeud (noeud dépot)
		randn1 = 1 + (rand() % (routes->at(randr1).nodes.size()-1));

		//cout << randr1 << " " << randn1 << endl;
		list<node_VRP*>::iterator it, it_prev, it_next, it2, it_next2;

		it = routes->at(randr1).nodes.begin();
		it_prev = routes->at(randr1).nodes.begin();
		it_next = routes->at(randr1).nodes.begin();

		advance(it,randn1);
		advance(it_prev,randn1-1);
		advance(it_next,randn1+1);

		//On récupère les routes ou on peut ajouter le noeud choisi au hasard.
		for(i=0; i < routes->size(); i++){
			if(i != randr1){
				if(routes->at(i).totalDemand() + (*it)->demand  <= capacity){
					possibleRoute.push_back(i);
				}
			}
		}

		if(possibleRoute.size()>0){
			randr2 = possibleRoute[rand() % possibleRoute.size()];
			randn2 = rand() % routes->at(randr2).nodes.size();

			it2 = routes->at(randr2).nodes.begin();
			it_next2 = routes->at(randr2).nodes.begin();

			advance(it2,randn2);
			advance(it_next2,randn2+1);

			//cout << randr2 << " " << randn2 << endl;

			float tempLength = routes->at(randr1).totalLength() + routes->at(randr2).totalLength();

			//cout << tempLength << endl;

			if(randn1 == routes->at(randr1).nodes.size()-1){
				tempLength -=G->distance((*it)->num, (*it_prev)->num);
				tempLength -=G->distance((*it)->num, 0);
				tempLength +=G->distance((*it_prev)->num, 0);
			}
			else{
				tempLength -=G->distance((*it)->num, (*it_prev)->num);
	                        tempLength -=G->distance((*it)->num, (*it_next)->num);
				tempLength +=G->distance((*it_prev)->num, (*it_next)->num);
			}
			if(randn2 == routes->at(randr2).nodes.size()-1){
				tempLength -=G->distance((*it2)->num, 0);
				tempLength +=G->distance((*it2)->num, (*it)->num);
				tempLength +=G->distance((*it)->num, 0);
			}
			else{
				tempLength -=G->distance((*it2)->num, (*it_next2)->num);
				tempLength +=G->distance((*it2)->num, (*it)->num);
				tempLength +=G->distance((*it)->num, (*it_next2)->num);
			}
			//cout << tempLength << endl;
			if(tempLength < routes->at(randr1).totalLength() + routes->at(randr2).totalLength()){
				routes->at(randr2).nodes.insert(++it2,(*it));
				routes->at(randr1).nodes.remove((*it));
				//cout << "improved at iteration " << nb_iter << endl;
				nb_iter = 0;
			}
		}
	}
}
