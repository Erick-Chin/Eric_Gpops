# Copyright (c) Yunus M. Agamawi and Anil Vithala Rao.  All Rights Reserved

# CHANGEME: This should be the name of your executable
# uncomment .exe if using Windows
EXT = #.exe
EXE = run_cgpops$(EXT)

# Specify path to IPOPT installation directory
IPOPT_DIR = ../../../Ipopt-3.12.13

# Specify path to APLOPS installation directory
CGPOPS_DIR = ../..

# CHANGEME: Here is the name of all object files corresponding to the source
#           code that you wrote in order to define the problem statement

CGPOPS_FUNC_DIR = $(CGPOPS_DIR)/src

OBJS_GOV = cgpops_wrapper.o cgpops_main.o cgpops_gov.o

_CGPOPS_OBJS = cgpopsClassFuncDef.o MatClasses.o ASCIItable.o Bicomplex.o HyperDual.o LGRClass.o cgpopsFuncDef1.o cgpopsFuncDef2.o cgpopsFuncDef3.o NLPDerivativesHD.o NLPDerivativesBC.o NLPDerivativesCD.o cgpops_nlp.o cgpopsMeshRef.o cgpopsHamiltonianDef.o find_polynomial_roots_jenkins_traub.o polynomial.o OCPFunctions.o

CGPOPS_OBJS = $(patsubst %,$(CGPOPS_FUNC_DIR)/%,$(_CGPOPS_OBJS))

OBJS = $(OBJS_GOV) $(OBJS_DS) $(CGPOPS_OBJS)

# CHANGEME: Additional libraries
ADDLIBS =

# CHANGEME: Additional flags for compilation (e.g., include flags)
ADDINCFLAGS = -I. -I$(CGPOPS_DIR) -I$(CGPOPS_FUNC_DIR) -I$(CGPOPS_DIR)/thirdparty/eigen


##########################################################################
#  Usually, you don't have to change anything below.  Note that if you   #
#  change certain compiler options, you might have to recompile Ipopt.   #
##########################################################################

# C++ Compiler command
CXX = g++
STD = c++11

# C++ Compiler options
# uncomment for warning flags
WARN_F =  #-DNDEBUG -Wparentheses -Wreturn-type -Wcast-qual -Wall -Wpointer-arith -Wwrite-strings -Wconversion
CXXFLAGS = -O3 -pipe $(WARN_F) -Wno-unknown-pragmas -Wno-long-long   -DIPOPT_BUILD -std=$(STD) -DSRC_PATH=\"$(CGPOPS_FUNC_DIR)\"

# additional C++ Compiler options for linking
CXXLINKFLAGS = 

# Include directories
INCL = `PKG_CONFIG_PATH=$(IPOPT_DIR)/lib64/pkgconfig:$(IPOPT_DIR)/lib/pkgconfig:$(IPOPT_DIR)/share/pkgconfig: pkg-config --cflags ipopt` $(ADDINCFLAGS)

# Linker flags
LIBS = $(LIB_STD_FS) `PKG_CONFIG_PATH=$(IPOPT_DIR)/lib64/pkgconfig:$(IPOPT_DIR)/lib/pkgconfig:$(IPOPT_DIR)/share/pkgconfig: pkg-config --libs ipopt`

# The following is necessary under cygwin, if native compilers are used
CYGPATH_W = echo

all: $(EXE)

.SUFFIXES: .cpp .c .o .obj .hpp .h

$(EXE): $(OBJS)
	bla=;\
	for file in $(OBJS); do bla="$$bla `$(CYGPATH_W) $$file`"; done; \
	$(CXX) $(CXXLINKFLAGS) $(CXXFLAGS) -o $@ $$bla $(ADDLIBS) $(LIBS); \

fast: $(EXE)
	./$(EXE)

dust:
	rm -rf ipoptINFO.txt *.m *.bin

sweep: dust
	rm -rf $(EXE) $(OBJS_DS) $(OBJS_GOV) $(DS_FILES) run_cgpops fast

clean: sweep
	rm -rf $(OBJS)

.cpp.o:
	$(CXX) $(CXXFLAGS) $(INCL) -c -o $@ $<

.cpp.obj:
	$(CXX) $(CXXFLAGS) $(INCL) -c -o $@ `$(CYGPATH_W) '$<'`

.c.o:
	$(CXX) $(CXXFLAGS) $(INCL) -c -o $@ $<

.c.obj:
	$(CXX) $(CXXFLAGS) $(INCL) -c -o $@ `$(CYGPATH_W) '$<'`

.cc.o:
	$(CXX) $(CXXFLAGS) $(INCL) -c -o $@ $<

.cc.obj:
	$(CXX) $(CXXFLAGS) $(INCL) -c -o $@ `$(CYGPATH_W) '$<'`
