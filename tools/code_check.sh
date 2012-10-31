#!/bin/sh

echo "*:gazebo/common/Plugin.hh:113" > /tmp/gazebo_cpp_check.suppress
echo "*:gazebo/common/Plugin.hh:114" >> /tmp/gazebo_cpp_check.suppress
echo "*:gazebo/sdf/interface/parser.cc:544" >> /tmp/gazebo_cpp_check.suppress
echo "*:gazebo/common/STLLoader.cc:94" >> /tmp/gazebo_cpp_check.suppress
echo "*:gazebo/common/STLLoader.cc:105" >> /tmp/gazebo_cpp_check.suppress
echo "*:gazebo/common/STLLoader.cc:126" >> /tmp/gazebo_cpp_check.suppress
echo "*:gazebo/common/STLLoader.cc:149" >> /tmp/gazebo_cpp_check.suppress

#cppcheck
cppcheck --enable=all -q --suppressions-list=/tmp/gazebo_cpp_check.suppress --xml `find ./gazebo ./tools ./plugins ./examples ./test/regression ./interfaces -name "*.cc"`

# cpplint
find ./gazebo ./tools ./plugins ./examples ./test/regression ./interfaces -print0 -name "*.cc" -o -name "*.hh" -o -name "*.c" -o -name "*.h" | xargs -0 python tools/cpplint.py 2>&1
