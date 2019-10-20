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
	$(CCC) -c $(CCFLAGS) $(CCLNDIRS) -c Graph/Graph_VRP.cpp -o Graph/Graph_VRP.o

Route.o: Route/Route_VRP.cpp Route/Route_VRP.h
	$(CCC) -c $(CCFLAGS) $(CCLNDIRS) -c Route/Route_VRP.cpp -o Route/Route_VRP.o 

Heuristic_VRP.o: Heuristic_VRP.cpp
	$(CCC) -c $(CCFLAGS) $(CCLNDIRS) Heuristic_VRP.cpp

Heuristic_VRP: Heuristic_VRP.o Graph.o Route.o
	$(CCC) $(CCFLAGS) $(CCLNDIRS) Heuristic_VRP.o Graph/Graph_VRP.o Route/Route_VRP.o -o Heuristic_VRP $(CCLNFLAGS)

clean:
	rm -f *.o Heuristic_VRP
