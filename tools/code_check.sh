#!/bin/sh

# Jenkins will pass -xml, in which case we want to generate XML output
xmlout=0
if test "$1" = "-xmldir" -a -n "$2"; then
  xmlout=1
  xmldir=$2
  mkdir -p $xmldir
  rm -rf $xmldir/*.xml
fi

echo "*:gazebo/common/Plugin.hh:113" > /tmp/gazebo_cpp_check.suppress
echo "*:gazebo/common/Plugin.hh:114" >> /tmp/gazebo_cpp_check.suppress
echo "*:gazebo/sdf/interface/parser.cc:544" >> /tmp/gazebo_cpp_check.suppress
echo "*:gazebo/common/STLLoader.cc:94" >> /tmp/gazebo_cpp_check.suppress
echo "*:gazebo/common/STLLoader.cc:105" >> /tmp/gazebo_cpp_check.suppress
echo "*:gazebo/common/STLLoader.cc:126" >> /tmp/gazebo_cpp_check.suppress
echo "*:gazebo/common/STLLoader.cc:149" >> /tmp/gazebo_cpp_check.suppress

#cppcheck
if [ $xmlout -eq 1 ]; then
  (cppcheck --enable=all -q --suppressions-list=/tmp/gazebo_cpp_check.suppress --xml `find ./gazebo ./tools ./plugins ./examples ./test/regression ./interfaces -name "*.cc"`) 2> $xmldir/cppcheck.xml
else
  cppcheck --enable=all -q --suppressions-list=/tmp/gazebo_cpp_check.suppress `find ./gazebo ./tools ./plugins ./examples ./test/regression ./interfaces -name "*.cc"`
fi

# cpplint
if [ $xmlout -eq 1 ]; then
  (find ./gazebo ./tools ./plugins ./examples ./test/regression ./interfaces -print0 -name "*.cc" -o -name "*.hh" -o -name "*.c" -o -name "*.h" | xargs -0 python tools/cpplint.py 2>&1) | python tools/cpplint_to_cppcheckxml.py 2> $xmldir/cpplint.xml
else
  find ./gazebo ./tools ./plugins ./examples ./test/regression ./interfaces -print0 -name "*.cc" -o -name "*.hh" -o -name "*.c" -o -name "*.h" | xargs -0 python tools/cpplint.py 2>&1
fi
