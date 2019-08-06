#!/bin/bash

export LD_LIBRARY_PATH=${LIBZMQ_FQ_DIR}/lib64:${LD_LIBRARY_PATH}
export PYTHONPATH=

ldd $UBCV_LIB/*.so
ldd $LARCV_LIBDIR/*.so
