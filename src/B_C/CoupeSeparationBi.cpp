#include <ilcplex/ilocplex.h>
#include <vector>
#include <math.h>
#include "../Graph/Graph_VRP.h"

#define epsilon 0.0001

using namespace::std;

//Circuit inequality separation algorithm when x is integer
void  find_ViolatedCoupeCstBi_INTEGER(IloEnv env, graph_VRP & G,  vector<vector<IloNumVar> >& x, vector<vector<float> > &fracsol, list<IloRange> & L_ViolatedCst, int capacity){

	vector<vector<int> > sol;
	list<int> L;
	int i,j,k;
	sol.resize(G.nb_nodes);

	// Some "integer" value of CPLEX are not exactly integer...
	for (i=0;i<G.nb_nodes;i++){
		sol[i].resize(G.nb_nodes);
		for (j=0; j<G.nb_nodes; j++){
			if(i != j){
				if (fracsol[i][j] > epsilon) sol[i][j]=1;
				else sol[i][j]=0;
			}
		}
	}

	vector<int> nodes_num;
	for(i=1; i<G.nb_nodes; i++){
		nodes_num.push_back(i);
	}

	//int count = pow(2,nodes_num.size());
	long count = pow(2,nodes_num.size());
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
				total_x += sol[setA[j]][setB[k]];
			}
			total_demand+=G.V_nodes[setA[j]].demand;
		}

		if(total_demand/capacity > total_x && setA.size()>0 && setB.size() >0){
			IloExpr expr(env);

	                for(j=0; j<setA.size(); j++){
	                        for(k=0; k<setB.size(); k++){
	                                expr += x[setA[j]][setB[k]];
	                        }
	                }

			IloRange newCte = IloRange(expr >= ceil(total_demand/capacity));
			//cout << newCte << endl;
			L_ViolatedCst.push_back(newCte);
			i = count;
		}
	}
}

