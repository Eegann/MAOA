#include <ilcplex/ilocplex.h>
#include <vector>
#include <math.h>
#include <typeinfo>
#include "../Graph/Graph_VRP.h"
#include "../ConstraintInfo/ConstraintInfo.h"
#define epsilon 0.0001

using namespace::std;

//Circuit inequality separation algorithm when x is integer
void  find_ViolatedCoupeCst_INTEGER(IloEnv env, graph_VRP & G,  vector<vector<IloNumVar> >& x, vector<vector<float> > &fracsol, list<IloRange> & L_ViolatedCst, int capacity){

	vector<vector<int> > sol;
	list<int> L;
	int j,k;
	long long i;
	sol.resize(G.nb_nodes);

	// Some "integer" value of CPLEX are not exactly integer...
	for (i=0;i<G.nb_nodes;i++){
		sol[i].resize(G.nb_nodes-i-1);
		for (j=0; j<G.nb_nodes-i-1; j++){
			if (fracsol[i][j] > 1+epsilon) sol[i][j]=2;
			else{
				if(fracsol[i][j] > epsilon) sol[i][j]=1;
				else sol[i][j]=0;
			}
		}
	}

	vector<int> nodes_num;
	for(i=1; i<G.nb_nodes; i++){
		nodes_num.push_back(i);
	}

	//int count = pow(2,nodes_num.size());
	long long count = pow(2,nodes_num.size());
	// The outer for loop will run (2^n)/2 times to have all subset (similar subsets above (2^n)/2).
	// Here variable i will act as a binary counter
	// starts at 2 to avoid void subsets
	for (i = 0; i < count; i++){
		vector<int> setA, setB;
		// The inner for loop will run n times , As the maximum number of elements a set can have is n
		// This loop will generate a subset
		// Starts at 1 to avoir having subsets with zero
		for (j = 0; j < nodes_num.size(); j++){
			// This if condition will check if jth bit in binary representation of  i  is set or not
			// if the value of (i & (1 << j)) is greater than 0 , include j in the first current subset
			// otherwise include j in the second subset
			if ((i & (1 << j)) > 0 )
				setA.push_back(nodes_num[j]);
			else
				setB.push_back(nodes_num[j]);
		}
		setB.push_back(0);
//		for(j=0; j<setA.size(); j++){
//			cout << setA[j] << " ";
//		}
//		cout << "\n";

		int total_x=0, total_xB=0;
		double total_demand=0;
		for(j=0; j<setA.size(); j++){
			for(k=0; k<setB.size(); k++){
				if(setA[j]<setB[k])
					total_x += sol[setA[j]][setB[k]-setA[j]-1];
				else
					total_x += sol[setB[k]][setA[j]-setB[k]-1];
			}
			total_demand+=G.V_nodes[setA[j]].demand;
		}

		if(2*total_demand/capacity > total_x && setA.size()>0 && setB.size() >0){
			IloExpr expr(env);

	                for(j=0; j<setA.size(); j++){
	                        for(k=0; k<setB.size(); k++){
					if(setA[j]<setB[k]){
		                                expr += x[setA[j]][setB[k]-setA[j]-1];
					}
					else{
						expr += x[setB[k]][setA[j]-setB[k]-1];
					}
	                        }
	                }

			IloRange newCte = IloRange(expr >= ceil(2*total_demand/capacity));
			L_ViolatedCst.push_back(newCte);
			i = count;
		}
	}
}
float compute_slack(vector<vector<float > > fracsol,ConstraintInfo* ci){
	int i, j;
	float totalx = 0;
	for(i =0; i< ci->v_i.size(); i++){
		totalx += fracsol[ci->v_i[i]][ci->v_j[i]];
	}
	return totalx - ci->v;
}

void greedy_shrinking(IloEnv env, graph_VRP & G,  vector<vector<IloNumVar> >& x, vector<vector<float> > &fracsol, list<IloRange> & L_ViolatedCst, list<IloRange> & L_RemoveCst, int capacity, list<ConstraintInfo*> &constraints, bool &uselist){
	int z;
	for(z=0; z<5; z++){
	srand(time(NULL));
	int i,j;
	int total_demand=G.V_nodes[1].demand;
	vector<int> setA,setB;
	int r = (rand()%(G.V_nodes.size()-1))+1;
	for(i=0;i<G.V_nodes.size();i++){
		if(i!=r)
			setB.push_back(i);
	}
	setA.push_back(r);
	float actualScore = 0;
	// Compute score of the set
	for(j = 0; j<G.V_nodes.size(); j++){
		if(setA[0] != j){
			if(setA[0]<j){
				actualScore += fracsol[setA[0]][j-setA[0]-1];
			}
			else{
				actualScore += fracsol[j][setA[0]-j-1];
			}
		}
	}
	while(setA.size()<G.V_nodes.size()-1){
		float bestScore = -100;
		int bestNode = -100;
		for(j=0; j<setB.size(); j++){
			if(setB[j]!=0){
				int score = actualScore;
				for(i = 0; i<setA.size(); i++){
					if(setA[i]<setB[j])
						score -= fracsol[setA[i]][setB[j]-setA[i]-1];
					else
						score -= fracsol[setB[j]][setA[i]-setB[j]-1];
				}
				for(i = 0; i<setB.size(); i++){
					if(setB[i]<setB[j])
						score += fracsol[setB[i]][setB[j]-setB[i]-1];
					if(setB[i]>setB[j])
						score += fracsol[setB[j]][setB[i]-setB[j]-1];
				}
				if(score>bestScore){
					bestScore=score;
					bestNode=setB[j];
				}
			}
		}
		setA.push_back(bestNode);
		setB.erase(std::remove(setB.begin(), setB.end(), bestNode), setB.end());
		total_demand+=G.V_nodes[bestNode].demand;
		actualScore=bestScore;
//		cout << "actualScore: " << actualScore << " actualSet: ";
//		for(i=0;i<setA.size();i++){
//			cout << setA[i] << " ";
//		}
//		cout << "otherSet: ";
//		for(i=0; i<setB.size(); i++){
//			cout << setB[i] << " ";
//		}
//		cout <<  "bestScore: " << bestScore << " bestNode: " << bestNode << endl;


		if(2*total_demand/capacity > actualScore + epsilon && setA.size()>1 && setB.size() >1){
			vector<int> v_i, v_j;
			IloExpr expr(env);

	                for(i=0; i<setA.size(); i++){
	                        for(j=0; j<setB.size(); j++){
					if(setA[i]<setB[j]){
		                                expr += x[setA[i]][setB[j]-setA[i]-1];
						if(uselist){
							v_i.push_back(setA[i]);
							v_j.push_back(setB[j]-setA[i]-1);
						}
					}
					else{
						expr += x[setB[j]][setA[i]-setB[j]-1];
                                                if(uselist){
                                                        v_i.push_back(setB[j]);
                                                        v_j.push_back(setA[i]-setB[j]-1);
                                                }
					}
	                        }
	                }
			IloRange newCte = IloRange(expr >= ceil(2*total_demand/capacity));
			L_ViolatedCst.push_back(newCte);

			if(uselist){
				ConstraintInfo* ci = new ConstraintInfo(newCte,v_i,v_j,ceil(2*total_demand/capacity));
				constraints.push_back(ci);
				if(constraints.size()>2000){
 					ConstraintInfo* toremove;
 					int value;
 					for(list<ConstraintInfo*>::iterator it = constraints.begin(); it != constraints.end(); it++){
 						if(it==constraints.begin()){
 							toremove=(*it);
 							value = compute_slack(fracsol, *it);
 						}
 						else{
 							int tempvalue = compute_slack(fracsol, *it);
 							if(tempvalue>value){
 								toremove=(*it);
 								value=tempvalue;
 							}
 						}
 					}
 					//cout << "value: "<<value<<endl;
 					if(value >0){
 						L_RemoveCst.push_back(toremove->cons);
 						constraints.remove(toremove);
 					}
 				}
//				list<ConstraintInfo*>::iterator it = constraints.begin();
//				while(it != constraints.end()){
//					if(compute_slack(fracsol, *it) > 5 + epsilon){
//						(*it)->age++;
//						if((*it)->age>100){
//							L_RemoveCst.push_back((*it)->cons);
//							it=constraints.erase(it);
//						}
//						else
//							it++;
//					}
//					else{
//						(*it)->age=0;
//						it++;
//					}
//				}
			}
		}
	}
	}
	cout << constraints.size() << endl;
}


