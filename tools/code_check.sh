#!/bin/sh

# Jenkins will pass -xml, in which case we want to generate XML output
xmlout=0
if test "$1" = "-xmldir" -a -n "$2"; then
  xmlout=1
  xmldir=$2
  mkdir -p $xmldir
  rm -rf $xmldir/*.xml
  # Assuming that Jenkins called, the `build` directory is a sibling to the src dir
  builddir=../build
else
  # This is a heuristic guess; not every developer puts the `build` dir in the src dir
  builddir=./build
fi

echo "*:gazebo/sdf/interface/parser.cc:544" >> /tmp/gazebo_cpp_check.suppress
echo "*:gazebo/common/STLLoader.cc:94" >> /tmp/gazebo_cpp_check.suppress
echo "*:gazebo/common/STLLoader.cc:105" >> /tmp/gazebo_cpp_check.suppress
echo "*:gazebo/common/STLLoader.cc:126" >> /tmp/gazebo_cpp_check.suppress
echo "*:gazebo/common/STLLoader.cc:149" >> /tmp/gazebo_cpp_check.suppress
echo "*:gazebo/common/Plugin.hh:145" >> /tmp/gazebo_cpp_check.suppress
echo "*:gazebo/common/Plugin.hh:118" >> /tmp/gazebo_cpp_check.suppress
echo "*:examples/plugins/custom_messages/custom_messages.cc:22" >> /tmp/gazebo_cpp_check.suppress


#cppcheck
if [ $xmlout -eq 1 ]; then
  # Run most of the checks in parallel
  (cppcheck --xml --enable=style,performance,portability,information --rule-file=./tools/cppcheck_rules/issue_581.rule -j 4 -q --suppressions-list=/tmp/gazebo_cpp_check.suppress `find ./plugins ./gazebo ./tools ./examples ./test/regression ./interfaces -name "*.cc"`) 2> $xmldir/cppcheck.xml 

  # Finally, check the configuration
  (cppcheck --xml --enable=missingInclude -q -j 4 --suppressions-list=/tmp/gazebo_cpp_check.suppress `find ./plugins ./gazebo ./tools ./examples ./test/regression ./interfaces -name "*.cc"` -I gazebo/rendering/skyx/include -I . -I $builddir -I $builddir/gazebo/msgs -I deps -I deps/opende/include -I test --check-config) 2> $xmldir/cppcheck-configuration.xml
else
  # Run most of the checks in parallel
  cppcheck --enable=style,performance,portability,information --rule-file=./tools/cppcheck_rules/issue_581.rule -j 4 -q --suppressions-list=/tmp/gazebo_cpp_check.suppress `find ./plugins ./gazebo ./tools ./examples ./test/regression ./interfaces -name "*.cc"` 2>&1

  # Finally, check the configuration
  cppcheck --enable=missingInclude -q -j 4 --suppressions-list=/tmp/gazebo_cpp_check.suppress `find ./plugins ./gazebo ./tools ./examples ./test/regression ./interfaces -name "*.cc"` -I gazebo/rendering/skyx/include -I . -I $builddir -I $builddir/gazebo/msgs -I deps -I deps/opende/include -I test --check-config 2>&1
fi

# cpplint
if [ $xmlout -eq 1 ]; then
  (find ./gazebo ./tools ./plugins ./examples ./test/regression ./interfaces -print0 -name "*.cc" -o -name "*.hh" -o -name "*.c" -o -name "*.h" | xargs -0 python tools/cpplint.py 2>&1) | python tools/cpplint_to_cppcheckxml.py 2> $xmldir/cpplint.xml
else
  find ./gazebo ./tools ./plugins ./examples ./test/regression ./interfaces -print0 -name "*.cc" -o -name "*.hh" -o -name "*.c" -o -name "*.h" | xargs -0 python tools/cpplint.py 2>&1
fi
