#!/bin/sh

echo "*:gazebo/common/Plugin.hh:93" > /tmp/gazebo_cpp_check.suppress
echo "*:gazebo/common/Plugin.hh:94" >> /tmp/gazebo_cpp_check.suppress
echo "*:gazebo/common/STLLoader.cc:94" >> /tmp/gazebo_cpp_check.suppress
echo "*:gazebo/common/STLLoader.cc:105" >> /tmp/gazebo_cpp_check.suppress
echo "*:gazebo/common/STLLoader.cc:126" >> /tmp/gazebo_cpp_check.suppress
echo "*:gazebo/common/STLLoader.cc:149" >> /tmp/gazebo_cpp_check.suppress

#cppcheck
#find . -name "*.cc" -print0 | xargs -0 cppcheck --enable=style --enable=performance --enable=portability --enable=missingInclude --enable=information -j 10 -q --suppressions-list=/tmp/gazebo_cpp_check.suppress
find ./gazebo ./tools ./plugins ./examples ./test/regression ./interfaces -name "*.cc" -print0 | xargs -0 cppcheck --enable=all -q --suppressions-list=/tmp/gazebo_cpp_check.suppress

# cpplint
find ./gazebo ./tools ./plugins ./examples ./test/regression ./interfaces -print0 -name "*.cc" -o -name "*.hh" -o -name "*.c" -o -name "*.h" | xargs -0 python tools/cpplint.py 2>&1 | grep -v skyx
