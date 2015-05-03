#!/bin/sh

# dependencies
pip2 install --user mpld3 jinja2 yattag

# jsbsim
git submodule init
git submodule update
cd external/jsbsim
mkdir build
cd build
cmake .. -DINSTALL_PYTHON_MODULE=ON -DPYTHON_EXECUTABLE=/usr/bin/python2 -DPYTHON_INCLUDE_DIR=/usr/include/python2.7 -DPYTHON_LIBRARY=/usr/lib/libpython2.7.so
make all -j8

echo "make sure to add $(pwd) to PYTHONPATH"

cd ../..

