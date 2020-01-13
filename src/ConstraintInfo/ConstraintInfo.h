#ifndef CONSTRAINT_INFO_H
#define CONSTRAINT_INFO_H
#include <ilcplex/ilocplex.h>
#include <vector>
using namespace std;
class ConstraintInfo{
	public:
	//Attributs
	IloRange cons;
	int age;
	float v;
	vector<int> v_i;
	vector<int> v_j;
	ConstraintInfo(IloRange p_cons, vector<int> ii, vector<int> jj, float vv);
};
#endif
