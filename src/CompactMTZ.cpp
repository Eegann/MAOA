#include <ilcplex/ilocplex.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <sstream>
#include "Graph/Graph_VRP.h"
#include "Route/Route_VRP.h"

#define epsilon 0.0001

using namespace std;

int main(int argc, char** argv){

	//////////////
	//////  DATA
	//////////////

        if(argc!=4){
                cerr << "usage: "<< argv[0]<<"\n\t instance_file \n\t nb_vehicure \n\t vehicule_capacity  "<<endl;
        }

        int i, j, nb_vehicules, capacity;
        string filename = argv[1];

        nb_vehicules = atoi(argv[2]);
        capacity = atoi(argv[3]);

        ifstream fic(filename.c_str());

        if (!fic){
                cerr<<"file "<<filename<<" not found"<<endl;
                return 1;
        }

        graph_VRP G;

        G.read_file(fic);

        fic.close();

        cout << "\n\tEnd reading\n" << endl;

	//////////////
	//////  CPLEX INITIALIZATION
	//////////////

	IloEnv env;
	IloModel model(env);

	////////////////////////
	//////  VAR
	////////////////////////

	cout << "Var" << endl;

	vector< vector<IloNumVar> > x;
	x.resize(G.nb_nodes);
	for(i=0;i<G.nb_nodes; i++){
		x[i].resize(G.nb_nodes);
		for(j=0; j<G.nb_nodes; j++){
			if(j != i){
				x[i][j]=IloNumVar(env, 0.0, 1.0, ILOINT);
				ostringstream varname;
				varname.str("");
				varname<<"x"<<i<<"_"<<j;
				x[i][j].setName(varname.str().c_str());
			}
		}
	}

	vector<IloNumVar> w;
	w.resize(G.nb_nodes);
	for(i=0;i<G.nb_nodes; i++){
		w[i]=IloNumVar(env, 0.0, capacity, ILOFLOAT);
		ostringstream varname;
		varname.str("");
		varname<<"w"<<i;
		w[i].setName(varname.str().c_str());
	}
	//////////////
	//////  CST
	//////////////

	cout << "Cst" << endl;

	IloRangeArray CC(env);
	int nbcst=0;

	// sum_{j=1}^{nb_nodes} x_{0,j}<=nb_vehicules
	IloExpr cst_route1(env);
	for(j=1; j<G.nb_nodes; j++){
		cst_route1+=x[0][j];
	}
	CC.add(cst_route1<=nb_vehicules);
	ostringstream cstname_route1;
	cstname_route1.str("");
	cstname_route1<<"Cst_nb_route_1";
	CC[nbcst].setName(cstname_route1.str().c_str());
	nbcst++;

	// sum_{i=1}^{nb_nodes} x_{i,0}<=nb_vehicules
        IloExpr cst_route2(env);
        for(i=1; i<G.nb_nodes; i++){
                cst_route2+=x[i][0];
        }
        CC.add(cst_route2<=nb_vehicules);
        ostringstream cstname_route2;
        cstname_route2.str("");
        cstname_route2<<"Cst_nb_route_2";
        CC[nbcst].setName(cstname_route2.str().c_str());
        nbcst++;

	// sum_{j=1}^{nb_nodes} x_{i,j} = 1 \forall i != 0
	// /!\ sum_{j=0}...
	for(i=1; i<G.nb_nodes; i++){
		IloExpr cst(env);
		for(j=0; j<G.nb_nodes; j++){
			if(i != j)
				cst+=x[i][j];
		}
		CC.add(cst==1);
	        ostringstream cstname;
	        cstname.str("");
	        cstname<<"Cst_flot_entrant_"<<i;
	        CC[nbcst].setName(cstname.str().c_str());
	        nbcst++;
	}
	// sum_{i=1}^{nb_nodes} x_{i,j} = 1 \forall j != 0
	// /!\ sum_{i=0}...
        for(j=1; j<G.nb_nodes; j++){
                IloExpr cst(env);
                for(i=0; i<G.nb_nodes; i++){
			if(i != j)
				cst+=x[i][j];
                }
                CC.add(cst==1);
                ostringstream cstname;
                cstname.str("");
                cstname<<"Cst_flot_sortant_"<<j;
                CC[nbcst].setName(cstname.str().c_str());
                nbcst++;
        }

	// w_i - w_j >= d_i - (capacity + d_i)(1-x_{i,j}) \forall (i,j) \in A
	// /!\ i>0...
	for(i=1;i<G.nb_nodes; i++){
		for(j=0; j<G.nb_nodes; j++){
			if(i != j){
				IloExpr cst(env);
				cst+=w[i] - w[j] - ( G.V_nodes[i].demand - (capacity + G.V_nodes[i].demand)*(1 - x[i][j]));
				CC.add(cst>=0);
			        ostringstream cstname;
			        cstname.str("");
			        cstname<<"Cst_MTZ_"<<i<<"_"<<j;
			        CC[nbcst].setName(cstname.str().c_str());
			        nbcst++;
			}
		}
	}

	model.add(CC);

	//////////////
	////// OBJ
	//////////////

	cout << "Obj" << endl;

	IloExpr objective(env);

	for(i=0; i<G.nb_nodes; i++){
		for(j=0; j<G.nb_nodes; j++){
			if(i != j)
				objective+=x[i][j]*G.distance(i,j);
		}
	}

	model.add(IloMinimize(env,objective));

	//////////////
	////// RESOLUTION
	//////////////

	cout << "Resolution" << endl;

	IloCplex cplex(model);
	cplex.setParam(IloCplex::TiLim, 300);
        cplex.setParam(IloCplex::Param::Threads, 1);
        cplex.setParam(IloCplex::Cliques,-1);
        cplex.setParam(IloCplex::Covers,-1);
        cplex.setParam(IloCplex::DisjCuts,-1);
        cplex.setParam(IloCplex::FlowCovers,-1);
        cplex.setParam(IloCplex::FlowPaths,-1);
        cplex.setParam(IloCplex::FracCuts,-1);
        cplex.setParam(IloCplex::GUBCovers,-1);
        cplex.setParam(IloCplex::ImplBd,-1);
        cplex.setParam(IloCplex::MIRCuts,-1);
        cplex.setParam(IloCplex::ZeroHalfCuts,-1);
        cplex.setParam(IloCplex::MCFCuts,-1);
        cplex.setParam(IloCplex::MIPInterval,1);
        cplex.setParam(IloCplex::HeurFreq,-1);
        cplex.setParam(IloCplex::ClockType,1);
        cplex.setParam(IloCplex::RINSHeur,-1);

	cplex.exportModel("../out/sortie_compact.lp");
	bool solved = cplex.solve();

        string fileout = "../out/CompactMTZ.csv";
        ofstream resultat(fileout.c_str(), ios::out | ios::app);

        resultat << filename << "," << cplex.getStatus() << ",";
        if(solved){
                resultat << cplex.getObjValue() << ",";
        }
        else{
                resultat << "-,";
        }

        resultat << cplex.getBestObjValue() << ",";

        if(solved)
                resultat << (cplex.getObjValue() - cplex.getBestObjValue())/cplex.getBestObjValue() << ",";
        else
                resultat << "-,";

        resultat << cplex.getTime() << "," << cplex.getNnodes() <<"\n";

	env.out() << "Solution status = " << cplex.getStatus() << endl;
	if(solved){
		env.out() << "Solution value = " << cplex.getObjValue() << endl;
		vector<vector<int > >solx;
		solx.resize(G.nb_nodes);
		for(i=0; i<G.nb_nodes; i++){
			solx[i].resize(G.nb_nodes);
			for(j=0;j<G.nb_nodes;j++){
				
				if(i != j){
					env.out() << "x_"<< i << "_" << j << " = " << cplex.getValue(x[i][j]);
					solx[i][j]=cplex.getValue(x[i][j]);
				}
				else
					solx[i][j]=0;
			}
			env.out() << endl;
		}
		G.toPDF(filename,solx,false);
	}
	else{
		env.out() << "No solution found" << endl;
	}

	env.end();



}
