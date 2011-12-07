#!/bin/sh

echo "unusedFunction" >> /tmp/gazebo_cpp_check.suppress

find . -name "*.cc" -exec cppcheck --enable=all --suppressions /tmp/gazebo_cpp_check.suppress -q -I ./src {} \;
