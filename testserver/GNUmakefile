

CXX=g++
CXXFLAGS=-g -fPIC

ZMQDIR=${MRB_INSTALL}/libzmq/install
CPPZMQDIR=${MRB_INSTALL}/cppzmq/install

INCLUDES=-I${ZMQDIR}/include -I${CPPZMQDIR}/include

LIBS=${ZMQDIR}/lib64/libzmq.a -lm -lpthread -lstdc++


all: start_testserver run_testclient


%: %.cxx
	$(CXX) $(CXXFLAGS) $(INCLUDES) $*.cxx -o $@ $(LIBS)

clean:
	@rm bin/*
