#!/bin/sh

echo "*:src/common/STLLoader.cc:91" > /tmp/gazebo_cpp_check.suppress
echo "*:src/common/STLLoader.cc:102" >> /tmp/gazebo_cpp_check.suppress
echo "*:src/common/STLLoader.cc:117" >> /tmp/gazebo_cpp_check.suppress
echo "*:src/common/STLLoader.cc:137" >> /tmp/gazebo_cpp_check.suppress

#cppcheck
find . -name "*.cc" -print0 | xargs -0 cppcheck --enable=style --enable=performance --enable=portability --enable=missingInclude --enable=information -j 10 -q --suppressions-list=/tmp/gazebo_cpp_check.suppress

# cpplint
find ./src ./test ./plugins ./player ./tools -print0 -name "*.cc" -o -name "*.hh" -o -name "*.c" -o -name "*.h" | xargs -0 python tools/cpplint.py
