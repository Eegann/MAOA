#include <ilcplex/ilocplex.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <sstream>

#include"Graph/Graph_VRP.h"

#define epsilon 0.00001

using namespace std;

#define OUTPUT

///////////////////////////////
///////////////////////////////

// Compact MIP "MTZ" formulation for the TSP
// Given a complete undirected graph G=(V,A)

// Min sum_{i=1 to n} sum{j=1 to n, i!=i}  c_ij x_ij
//   sum_{j=1 to n, j!=i} x_ij = 1   for all node i=1 to n
//   sum_{i=1 to n, i!=j} x_ij = 1   for all node j=1 to n
//   u_i -u_j + 1 <= n (1 -x_ij) for all i=2 to n and j = 2 to n j!= i
//   u_1=1
//   1<=u_i<=n for all i =1 to n
//   u_i real for all i =1 to n
//   x_ij in {0,1} for every arc (i,j) i=1 to n, j=1 to n i!=j

///////////////////////////////
///////////////////////////////
//Separation//
// Usefull inequalities 
bool  find_ViolatedMengerCutCst(IloEnv env, VRP_Graph & G,  vector<vector<IloNumVar> >& x,  IloRange & ViolatedCst){

  int i,j;
  list<int> W;
  list<int>::const_iterator it,it2;
  vector<int> V_W;
  float test;

  V_W.resize(G.nb_nodes);

  // Find a minimum cut


  test=G.Directed_MinimumCut(W);

  //cout<<"test = "<<test<<endl;
  
  if (test<1-epsilon && W.size()>=2) {
    // Found a violated inequality 
      
    IloExpr expr(env);
    
    bool Wclient=true;
    for (it=W.begin();it!=W.end();it++){
    	if (*it==0){
    		Wclient=false;
    		break;
    	}
    }
    for (i=0;i<G.nb_nodes;i++){
    if (!Wclient) V_W[i]=0;
    else V_W[i]=1;
	}


    

    for (it=W.begin();it!=W.end();it++) {
    if (!Wclient) V_W[*it]=1;
    else V_W[*it]=0;
    }

//     cout<<"Set W found :";
//     for (it=W.begin();it!=W.end();it++)
//       cout<<*it<<" ";
//     cout<<endl;
//     for (vector<int >::iterator it3=V_W.begin();it3!=V_W.end();it3++)
//       cout<<*it3<<" ";
//     cout<<endl;

    for (i=0;i<G.nb_nodes;i++)
      for (j=0;j<G.nb_nodes;j++)
    	if (V_W[i]==0 && V_W[j]==1 ) expr+=x[i][j];

			
      
    ViolatedCst=IloRange(expr >= 1);
    return true;
  }
  else
    return false;

}

// Usefull inequalities 
ILOUSERCUTCALLBACK2(UsercutMengerCutSeparation,VRP_Graph &, G,vector<vector<IloNumVar> >&,x){
#ifdef OUTPUT
  //cout<<"********* UserCut separation Callback *************"<<endl;
  #endif

  int i;
  list<C_link *>::const_iterator it;
  IloRange ViolatedCst;
  
  // Put the linear relaxation values on the edges of graph G

  for (i=0;i<G.nb_nodes;i++){
    for (it=G.V_nodes[i].L_adjLinks.begin();it!=G.V_nodes[i].L_adjLinks.end();it++){
    	int j=(*it)->return_other_extrem(i);
    	(*it)->algo_cost=getValue(x[i][j]);
    	//cout << i<<" "<< j<<" " <<getValue(x[i][j])<<endl;
			if((*it)->algo_cost<epsilon)(*it)->algo_cost=0 ;
      }
    }
    

  /* Separation of Cut inequalities */

  if (find_ViolatedMengerCutCst(getEnv(),G,x, ViolatedCst)){

    #ifdef OUTPUT
    //cout << "Adding constraint : "<<endl;
    //cout<< ViolatedCst << endl;
    #endif
    add(ViolatedCst,IloCplex::UseCutFilter);   // UseCutForce UseCutPurge UseCutFilter
  }
  #ifdef OUTPUT
    else {
//      cout<<"No Cst found"<<endl;
    }
  #endif
}


int main (int argc, char**argv){

  string name, nameext, nameextsol;
  int i,j,k;

  vector<int> sol;


  //////////////
  //////  DATA
  //////////////

  if(argc!=2){
    cerr<<"usage: "<<argv[0]<<" <TSP file name>   (without .vrp)"<<endl; 
    return 1;
  }

  name=argv[1];
  nameext=name+".vrp";
  nameextsol=name+".vrpsol";

  ifstream fic(nameext.c_str());

  if (fic==NULL){
    cerr<<"file "<<nameext<<" not found"<<endl;
    return 1;
  }

  VRP_Graph G;

  G.read_directed_VRP(fic);

  fic.close();


  //////////////
  //////  CPLEX INITIALIZATION
  //////////////


  IloEnv   env;
  IloModel model(env);



  ////////////////////////
  //////  VAR
  ////////////////////////


  vector<vector<IloNumVar> > x;
  
  
  x.resize(G.nb_nodes);
  
  int cpt=0;
  for (i=0;i<G.nb_nodes;i++)
    x[i].resize(G.nb_nodes);
  	cpt+=1;
  	
  	
  	
  for (i=0;i<G.nb_nodes;i++){
    for (j=0;j<G.nb_nodes;j++) {
    	if(i!=j){
				x[i][j]=IloNumVar(env, 0.0, 1.0, ILOINT);
				ostringstream varname;
				varname.str("");
				varname<<"x_"<<i<<"_"<<j;
				x[i][j].setName(varname.str().c_str());
				//cout << x[i][j]<<endl;
				}
    }
  }


  vector<IloNumVar> w;
  w.resize(G.nb_nodes);
  for(i = 1; i < G.nb_nodes; i++) {
    w[i]=IloNumVar(env,G.demand[i] ,G.maxCap, ILOFLOAT);
    ostringstream nomvar;
    nomvar.str("");
    nomvar<<"w_"<<i;
    w[i].setName(nomvar.str().c_str());
  }


  
  //////////////
  //////  CST
  //////////////

  IloRangeArray CC(env);
  int nbcst=0;
  //Contrainte 1 et 2
  IloExpr c1(env);
	for (j=1;j<G.nb_nodes;j++){
		c1+=x[0][j];
	}
	CC.add(c1<=G.nbTruck);
	
	IloExpr c2(env);
	for (j=1;j<G.nb_nodes;j++){
		c2+=x[j][0];
	}
	CC.add(c2<=G.nbTruck);
	


	nbcst=2;


  // Contraite 3 et 4 sum_{j=1 to n, j!=i} x_ij = 1   for all node i=1 to n
  for (i=1;i<G.nb_nodes;i++){
    IloExpr c1(env);
    for (j=0;j<G.nb_nodes;j++){
    	if (j!=i){
				c1+=x[i][j];
				//cout<<x[i][j]<< "+ ";
			}
		}
		//cout<<endl;
    CC.add(c1==1);
    ostringstream nomcst;
    nomcst.str("");
    nomcst<<"CstDeg"<<i;
    CC[nbcst].setName(nomcst.str().c_str());
    nbcst++;
  }
    for (j=1;j<G.nb_nodes;j++){
    IloExpr c1(env);
    for (i=0;i<G.nb_nodes;i++){
    	if (j!=i){
				c1+=x[i][j];
				//cout<<x[i][j]<< "+ ";
			}
		}
		//cout<<endl;
    CC.add(c1==1);
    ostringstream nomcst;
    nomcst.str("");
    nomcst<<"CstDeg"<<i;
    CC[nbcst].setName(nomcst.str().c_str());
    nbcst++;
  }
  


//MTZ constraints

  for (i=1;i<G.nb_nodes;i++){
    for (j=1;j<G.nb_nodes;j++){
  		IloExpr c2(env);
  		if (i!=j){
				c2=w[i]-w[j]-G.demand[i] + (G.maxCap+G.demand[i])*(1-x[i][j]);
				CC.add(c2>= 0);
				ostringstream nomcst;
				nomcst.str("");
				nomcst<<"CstMTZ_"<<i<<"_"<<j;
				CC[nbcst].setName(nomcst.str().c_str());
				nbcst++;
			}
    }
  }



  model.add(CC);
  
  
	IloCplex cplex(model);
	//cplex.setParam(IloCplex::TiLim,50);
	//cplex.use(LazyMengerCutSeparation(env,G,x));
	cplex.use(UsercutMengerCutSeparation(env,G,x));
  //////////////
  ////// OBJ
  //////////////

	
  
  IloObjective obj=IloAdd(model, IloMinimize(env, 0.0));
  
  for (i=0;i<G.nb_nodes;i++)
    for (j=0;j<G.nb_nodes;j++)
    	if (i!=j)
			obj.setLinearCoef(x[i][j],G.lengthTSP(i,j));
 

  ///////////
  //// RESOLUTION
  //////////



  // cplex.setParam(IloCplex::Cliques,-1);
  // cplex.setParam(IloCplex::Covers,-1);
  // cplex.setParam(IloCplex::DisjCuts,-1);
  // cplex.setParam(IloCplex::FlowCovers,-1);
  // cplex.setParam(IloCplex::FlowPaths,-1);
  // cplex.setParam(IloCplex::FracCuts,-1);
  // cplex.setParam(IloCplex::GUBCovers,-1);
  // cplex.setParam(IloCplex::ImplBd,-1);
  // cplex.setParam(IloCplex::MIRCuts,-1);
  // cplex.setParam(IloCplex::ZeroHalfCuts,-1);
  // cplex.setParam(IloCplex::MCFCuts,-1);
  // cplex.setParam(IloCplex::MIPInterval,1);
  // cplex.setParam(IloCplex::HeurFreq,-1);
  // cplex.setParam(IloCplex::ClockType,1);
  // cplex.setParam(IloCplex::RINSHeur,-1);


  #ifdef OUTPUT
  cout<<"Wrote LP on file"<<endl;
  cplex.exportModel("sortie.lp");
  #endif

  if ( !cplex.solve() ) {
    env.error() << "Failed to optimize LP" << endl;
    exit(1);
  }

 
  env.out() << "Solution status = " << cplex.getStatus() << endl;
  env.out() << "Solution value  = " << cplex.getObjValue() << endl;


  vector<pair<int,int> >   Lsol;
  for(i = 0; i < G.nb_nodes; i++)
     for (j=0;j<G.nb_nodes;j++)
			if (i!=j && cplex.getValue(x[i][j])>1-epsilon ){
				cout<<"chemin "<<i<<" "<<j<<endl;
	  		Lsol.push_back(make_pair(i,j));
	  	}
	for (i = 1; i < G.nb_nodes; i++){
		cout<<"w["<< i <<"]= "<< cplex.getValue(w[i])<<"   " <<G.demand[i]<<endl;
	}
	  	



  //////////////
  //////  CPLEX's ENDING
  //////////////

  env.end();

  //////////////
  //////  OUTPUT
  //////////////


  vector<pair<int,int> >::const_iterator itp;
 
  ofstream ficsol(nameextsol.c_str());
  double best_length=0;
  for(itp = Lsol.begin(); itp!=Lsol.end();itp++) {
    best_length+=G.lengthTSP(itp->first,itp->second);
    ficsol<<itp->first<<" "<<itp->second<<endl;
  }
 
  ficsol.close();

  cout<<"Tour found of value : "<<best_length<<endl;
  
  cout<<endl;
  //affichage
  vector<int> nodes;
  nodes.resize(G.nb_nodes);
  for(int no=0;no<G.nb_nodes;no++) nodes[no]=1;
	vector<pair<int,int> >   Ls=Lsol;
	pair<int,int> start=Ls[0];
	pair<int,int> v;
	int u=start.second;
	Ls.erase(Ls.begin());
	cout<<start.first<<" ";
	while(Ls.size()>0){
		//cout<<u<<"  ";
		for (vector<pair<int,int> >::iterator it=Ls.begin(); it!=Ls.end();it++){
			if (it->first==u) {
				v=*it;
				cout<<u<<" ";
				u=v.second;
				Ls.erase(it);
				if (u==start.first && Ls.size()!=0){
					cout<<endl;
					start=Ls[0];
					
					Ls.erase(Ls.begin());
					
					cout<<start.first <<" ";
					u=start.second;
				}
				break;
			}
			if (it->second==u) {
				v=*it;
				cout<<u<<" ";
				u=v.first;
				Ls.erase(it);
				if (u==start.first && Ls.size()!=0){
					cout<<endl;
					start=Ls[0];
					Ls.erase(Ls.begin());
					
					cout<<start.first <<" ";
					u=start.second;
				}
				break;
			}
			
		
		}
		
	
	}



  return 0;
}