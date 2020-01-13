#include "ConstraintInfo.h"
#include <ilcplex/ilocplex.h>
#include <vector>
ConstraintInfo::ConstraintInfo(IloRange p_cons, vector<int> ii, vector<int> jj, float vv){
	cons = p_cons;
	v_i = ii;
	v_j = jj;
	v = vv;
	age=0;
};
