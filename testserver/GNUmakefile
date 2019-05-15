CXX=g++
CXXFLAGS=-g -fPIC

ZMQDIR=${MRB_INSTALL}/libzmq/install
CPPZMQDIR=${MRB_INSTALL}/cppzmq/install

INCLUDES=-I${LIBZMQ_FQ_DIR}/include -I${CPPZMQ_DIR}/include

LIBS=-L${LIBZMQ_FQ_DIR}/lib64 -lzmq -lm -lpthread -lstdc++


all: start_testserver run_testclient


%: %.cxx
	$(CXX) $(CXXFLAGS) $(INCLUDES) $*.cxx -o $@ $(LIBS)

clean:
	@rm bin/*
