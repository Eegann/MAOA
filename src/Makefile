#------------------------------------------------------------
#
# Please define an environnement variable PATHTUTOMIP
# equal to the XXX directory where Lemon,Graphviz and Cplex have been installed
# In order to these:
# edit ~/.bashrc add "export PATHTUTOMIP=XXX"
# close your xterm and star new ones
#
#------------------------------------------------------------

CPLEXDIR      = $(PATHTUTOMIP)/cplex-12.7.1/cplex
CONCERTDIR    = $(PATHTUTOMIP)/cplex-12.7.1/concert
LEMONDIR      = $(PATHTUTOMIP)/lemon-1.3.1


SYSTEM     = x86-64_linux
LIBFORMAT  = static_pic


# ---------------------------------------------------------------------
# Compiler selection 
# ---------------------------------------------------------------------

CCC = g++ -O0


# ---------------------------------------------------------------------
# Compiler options 
# ---------------------------------------------------------------------

CCOPT = -m64 -O -fPIC -fno-strict-aliasing -fexceptions -DNDEBUG -DIL_STD -Wno-ignored-attributes

# ---------------------------------------------------------------------
# Link options and libraries
# ---------------------------------------------------------------------

CPLEXBINDIR   = $(CPLEXDIR)/bin/$(BINDIST)
CPLEXLIBDIR   = $(CPLEXDIR)/lib/$(SYSTEM)/$(LIBFORMAT)
CONCERTLIBDIR = $(CONCERTDIR)/lib/$(SYSTEM)/$(LIBFORMAT)
LEMONLIBDIR	= $(LEMONDIR)/lib

CCLNDIRS  = -L$(CPLEXLIBDIR) -L$(CONCERTLIBDIR) -L$(LEMONLIBDIR)


CONCERTINCDIR = $(CONCERTDIR)/include
CPLEXINCDIR   = $(CPLEXDIR)/include
LEMONINCDIR   = $(LEMONDIR)/include

CCLNFLAGS = -lconcert -lilocplex -lcplex -lemon -lm -lpthread

CCFLAGS = $(CCOPT) -I$(CPLEXINCDIR) -I$(CONCERTINCDIR) -I$(LEMONINCDIR)




# ---------------------------------------------------------------------
# Your part
# ---------------------------------------------------------------------



all: Heuristic_VRP

Graph.o: Graph/Graph_VRP.cpp Graph/Graph_VRP.h
	$(CCC) -c $(CCFLAGS) $(CCLNDIRS) -c Graph/Graph_VRP.cpp -o bin/Graph_VRP.o

Route.o: Route/Route_VRP.cpp Route/Route_VRP.h
	$(CCC) -c $(CCFLAGS) $(CCLNDIRS) -c Route/Route_VRP.cpp -o bin/Route_VRP.o 

CoupeSeparationUni.o: B_C/CoupeSeparationUni.cpp
	$(CCC) -c $(CCFLAGS) $(CCLNDIRS) -c B_C/CoupeSeparationUni.cpp -o bin/CoupeSeparationUni.o

CoupeSeparationBi.o: B_C/CoupeSeparationBi.cpp
	$(CCC) -c $(CCFLAGS) $(CCLNDIRS) -c B_C/CoupeSeparationBi.cpp -o bin/CoupeSeparationBi.o

Heuristic_VRP.o: Heuristic_VRP.cpp
	$(CCC) -c $(CCFLAGS) $(CCLNDIRS) Heuristic_VRP.cpp -o bin/Heuristic_VRP.o

ConstraintInfo.o: ConstraintInfo/ConstraintInfo.cpp
	$(CCC) -c $(CCFLAGS) $(CCLNDIRS) ConstraintInfo/ConstraintInfo.cpp -o bin/ConstraintInfo.o

CompactMTZ.o: CompactMTZ.cpp
	$(CCC) -c $(CCFLAGS) $(CCLNDIRS) CompactMTZ.cpp -o bin/CompactMTZ.o

B_CUni.o: B_C/B_CUni.cpp
	$(CCC) -c $(CCFLAGS) $(CCLNDIRS) B_C/B_CUni.cpp -o bin/B_CUni.o

B_CBi.o: B_C/B_CBi.cpp
	$(CCC) -c $(CCFLAGS) $(CCLNDIRS) B_C/B_CBi.cpp -o bin/B_CBi.o

Greedy.o: Algo/Greedy.cpp
	$(CCC) -c $(CCFLAGS) $(CCLNDIRS) Algo/Greedy.cpp -o bin/Greedy.o

Heuristic_VRP: Heuristic_VRP.o Graph.o Route.o
	$(CCC) $(CCFLAGS) $(CCLNDIRS) bin/Heuristic_VRP.o bin/Graph_VRP.o bin/Route_VRP.o -o Heuristic_VRP $(CCLNFLAGS)

CompactMTZ: CompactMTZ.o Graph.o Route.o
	$(CCC) $(CCFLAGS) $(CCLNDIRS) bin/CompactMTZ.o bin/Graph_VRP.o bin/Route_VRP.o -o CompactMTZ $(CCLNFLAGS)

B_CUni: B_CUni.o CoupeSeparationUni.o Graph.o Route.o ConstraintInfo.o
	$(CCC) $(CCFLAGS) $(CCLNDIRS) bin/B_CUni.o bin/CoupeSeparationUni.o bin/Graph_VRP.o bin/Route_VRP.o bin/ConstraintInfo.o -o B_CUni $(CCLNFLAGS)

B_CBi: B_CBi.o CoupeSeparationBi.o Graph.o Route.o
	$(CCC) $(CCFLAGS) $(CCLNDIRS) bin/B_CBi.o bin/CoupeSeparationBi.o bin/Graph_VRP.o bin/Route_VRP.o -o B_CBi $(CCLNFLAGS)


clean:
	rm -f *.o Heuristic_VRP
