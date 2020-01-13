#include <ilcplex/ilocplex.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <sstream>
#include "../Graph/Graph_VRP.h"
#include "../Route/Route_VRP.h"
#include "../ConstraintInfo/ConstraintInfo.h"

//#define OUTPUT

using namespace std;

struct Args {
	vector<vector<IloNumVar> > x;
	int capacity;
	long n;
	list<ConstraintInfo*> constraints;
	bool uselist;
	IloModel* model;
};

///////////////////////////////
////// SEPARATION FCT ////////
/////////////////////////////

// declaration of the functions that are dispatched outstide this file
void find_ViolatedCoupeCst_INTEGER(IloEnv env, graph_VRP & G,  vector<vector<IloNumVar> >& x, vector<vector<float> >&fracsol, list<IloRange> & L_ViolatedCst, int capacity);

void find_ViolatedCoupeCst(IloEnv env, graph_VRP & G,  vector<vector<IloNumVar> >& x, vector<vector<float> >&fracsol, list<IloRange> & L_ViolatedCst, int capacity);

void greedy_shrinking(IloEnv env, graph_VRP & G,  vector<vector<IloNumVar> >& x, vector<vector<float> >&fracsol, list<IloRange> & L_ViolatedCst, list<IloRange> &L_RemoveCst, int capacity, list<ConstraintInfo*> &constraints, bool &uselist);



// Cplex callback that needs to be in the same file as the main one

// Necessary inequalitie
ILOLAZYCONSTRAINTCALLBACK2(LazyCoupeSeparation,
                           graph_VRP &, G,
                           Args & , args
                    ){
	#ifdef OUTPUT
	cout<<"*********** Lazy separation Callback *************"<<endl;
	#endif

	int i,j;
	vector<vector<float> > fracsol;
	list<IloRange> L_ViolatedCst;
	fracsol.resize(G.nb_nodes);
        for(i=0;i<G.nb_nodes; i++){
                fracsol[i].resize(G.nb_nodes-i-1);
                for(j=0; j<G.nb_nodes-i-1; j++){
			fracsol[i][j] = getValue(args.x[i][j]);
                }
        }

	/* Separation of Circuit inequalities */

	L_ViolatedCst.clear();
	find_ViolatedCoupeCst_INTEGER(getEnv(), G, args.x, fracsol, L_ViolatedCst, args.capacity);

	#ifdef OUTPUT
	if (L_ViolatedCst.empty()) cout<<"No Cst found"<<endl;
	#endif

	while (!L_ViolatedCst.empty()){
		#ifdef OUTPUT
		cout << "Adding constraint : " << L_ViolatedCst.front() << endl;
		#endif
		add(L_ViolatedCst.front(),IloCplex::UseCutForce); //UseCutPurge);
		L_ViolatedCst.pop_front();
	}
}

ILOUSERCUTCALLBACK2(UserCoupeSeparation,
                           graph_VRP &, G,
                           Args & , args
                    ){
	if(args.n%3 ==0 && getNnodes()!=0){
	        #ifdef OUTPUT
	        cout<<"*********** User separation Callback *************"<<endl;
	        #endif
	        int i,j;
	        vector<vector<float> > fracsol;
	        list<IloRange> L_ViolatedCst;
		list<IloRange> L_RemoveCst;
	        fracsol.resize(G.nb_nodes);
	        for(i=0;i<G.nb_nodes; i++){
	                fracsol[i].resize(G.nb_nodes-i-1);
	                for(j=0; j<G.nb_nodes-i-1; j++){
	                        fracsol[i][j] = getValue(args.x[i][j]);
	                }
	        }

	        /* Separation of Circuit inequalities */
	        L_ViolatedCst.clear();
		L_RemoveCst.clear();
	        greedy_shrinking(getEnv(), G, args.x, fracsol, L_ViolatedCst, L_RemoveCst, args.capacity, args.constraints, args.uselist);

	        #ifdef OUTPUT
	        if (L_ViolatedCst.empty()) cout<<"No Cst found"<<endl;
	        #endif
		while(!L_RemoveCst.empty()){
			args.model->remove(L_RemoveCst.front());
//			find(L_ViolatedCst.begin(),L_ViolatedCst.end(),L_RemoveCst.front());
//			L_ViolatedCst.remove(L_RemoveCst.front());
			L_RemoveCst.pop_front();
		}
	        while (!L_ViolatedCst.empty()){
	                #ifdef OUTPUT
	                cout << "Adding constraint : " << L_ViolatedCst.front() << endl;
	                #endif
	                add(L_ViolatedCst.front(),IloCplex::UseCutForce); //UseCutPurge);
	                L_ViolatedCst.pop_front();
	        }
	}
	args.n++;
}


int main(int argc, char** argv){

        //////////////
        //////  DATA
        //////////////

        if(argc!=5){
                cerr << "usage: "<< argv[0]<<"\n\t instance_file \n\t nb_vehicure \n\t vehicule_capacity \n\t b (branch/bound) c (branch/cut) l (branch/cut with list of constraints)"<<endl;
        }

        int i, j, nb_vehicules, capacity;
	char* type;
        string filename = argv[1];

        nb_vehicules = atoi(argv[2]);
        capacity = atoi(argv[3]);

	type=argv[4];

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

        vector< vector<IloNumVar> > x;
        x.resize(G.nb_nodes);
        for(i=0;i<G.nb_nodes; i++){
                x[i].resize(G.nb_nodes-i-1);
                for(j=0; j<G.nb_nodes-i-1; j++){
			if(i==0)
	                        x[i][j]=IloNumVar(env, 0.0, 2.0, ILOINT);
			else
				x[i][j]=IloNumVar(env, 0.0, 1.0, ILOINT);
                        ostringstream varname;
                        varname.str("");
                        varname<<"x"<<i<<"_"<<(j+i+1);
                        x[i][j].setName(varname.str().c_str());
                }
        }
        //////////////
        //////  CST
        //////////////

        cout << "Cst" << endl;

        IloRangeArray CC(env);
        int nbcst=0;

        // sum_{j=1}^{nb_nodes} x_{0,j}<=2*nb_vehicules
        IloExpr cst_route1(env);
        for(j=0; j<G.nb_nodes-1; j++){
                cst_route1+=x[0][j];
        }
        CC.add(cst_route1<=2*nb_vehicules);
        ostringstream cstname_route1;
        cstname_route1.str("");
        cstname_route1<<"Cst_nb_route";
        CC[nbcst].setName(cstname_route1.str().c_str());
        nbcst++;

        // sum_{j=1}^{nb_nodes} x_{i,j} = 2 \forall i != 0
        // /!\ sum_{j=0}...

        for(i=1; i<G.nb_nodes; i++){
                IloExpr cst(env);
                for(j=0; j<G.nb_nodes-i-1; j++){
                        cst+=x[i][j];
                }
		for(j=0; j<i; j++){
			cst+=x[j][i-1-j];
		}
                CC.add(cst==2);
                ostringstream cstname;
                cstname.str("");
                cstname<<"Cst_flot_"<<i;
                CC[nbcst].setName(cstname.str().c_str());
                nbcst++;
        }

	model.add(CC);

	//////////////
	///// OBJ
        //////////////

        cout << "Obj" << endl;

        IloExpr objective(env);

        for(i=0; i<G.nb_nodes; i++){
                for(j=0; j<G.nb_nodes-i-1; j++){
                        objective+=x[i][j]*G.distance(i,j+i+1);
                }
        }

        model.add(IloMinimize(env,objective));

        //////////////
        ////// RESOLUTION
        //////////////

        cout << "Resolution" << endl;

	IloCplex cplex(model);

	//ADD SEPARATION CALLBACK
	Args args;
	args.x = x;
	args.capacity = capacity;
	args.n=0;
	args.model=&model;
	if(*type=='l')
		args.uselist=1;
	else
		args.uselist=0;
	cplex.use(LazyCoupeSeparation(env,G,args));
	if(*type=='c' || *type=='l')
		cplex.use(UserCoupeSeparation(env,G,args));

	cplex.setParam(IloCplex::TiLim, 300);
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


        cplex.exportModel("../out/sortie_b_c_uni.lp");
        bool solved = cplex.solve();

	string fileout;
	if(*type=='c')
		fileout = "../out/B_CUni.csv";
	else{
		if(*type=='l'){
			fileout = "../out/B_CUni_l.csv";
			cout << args.constraints.size() << endl;
		}
		else
			fileout = "../out/B_BUni.csv";
	}
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
		vector<vector< int > >solx;
		solx.resize(G.nb_nodes);
		for(i=0; i<G.nb_nodes; i++){
			solx[i].resize(G.nb_nodes-i-1);
			for(j=0;j<G.nb_nodes-i-1;j++){
				env.out() << "x_"<< i << "_" << j+i+1 << " = " << cplex.getValue(x[i][j]);
				solx[i][j]=cplex.getValue(x[i][j]);
			}
			env.out() << endl;
		}
		G.toPDF(filename,solx,true);
        }
        else{
                env.out() << "No solution found" << endl;
        }

        env.end();

}
