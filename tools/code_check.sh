#!/bin/sh

echo "unusedFunction" >> /tmp/gazebo_cpp_check.suppress

find ./src -name "*.cc" -exec cppcheck --enable=all --suppressions /tmp/gazebo_cpp_check.suppress -q -I ./src -I ./build -I ./build/src -I ./build/src/msgs -I ./deps/opende/include/ {} \;
