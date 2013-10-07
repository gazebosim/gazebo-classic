#!/bin/sh

# Jenkins will pass -xml, in which case we want to generate XML output
xmlout=0
if test "$1" = "-xmldir" -a -n "$2"; then
  xmlout=1
  xmldir=$2
  mkdir -p $xmldir
  rm -rf $xmldir/*.xml
  # Assuming that Jenkins called, the `build` directory is sibling to src dir
  builddir=../build
else
  # This is a heuristic guess; not everyone puts the `build` dir in the src dir
  builddir=./build
fi

QUICK_CHECK=0
if test "$1" = "--quick"
then
  QUICK_CHECK=1
  QUICK_SOURCE=$2
  hg_root=`hg root`
  if [ "$?" -ne "0" ] ; then
    echo This is not an hg repository
    exit
  fi
  cd $hg_root
  hg log -r $QUICK_SOURCE > /dev/null
  if [ "$?" -ne "0" ] ; then
    echo $QUICK_SOURCE is not a valid changeset hash
    exit
  fi
  CHECK_FILES=""
  while read line; do
    for f in $line; do
      CHECK_FILES="$CHECK_FILES `echo $f | grep '\.[ch][ch]*$'`"
    done
  done
  CPPCHECK_FILES="$CHECK_FILES"
  CPPLINT_FILES="$CHECK_FILES"
  QUICK_TMP=`mktemp -t asdfXXXXXXXXXX`
else
  CHECK_DIRS="./plugins ./gazebo ./tools ./examples ./test/integration"\
" ./test/regression ./interfaces ./test/performance"\
" ./test/cmake ./test/pkgconfig ./test/ServerFixture.*"
  CPPCHECK_FILES=`find $CHECK_DIRS -name "*.cc"`
  CPPLINT_FILES=`\
    find $CHECK_DIRS -name "*.cc" -o -name "*.hh" -o -name "*.c" -o -name "*.h"`
fi

SUPPRESS=/tmp/gazebo_cpp_check.suppress
echo "*:gazebo/sdf/interface/parser.cc:544" > $SUPPRESS
echo "*:gazebo/common/STLLoader.cc:94" >> $SUPPRESS
echo "*:gazebo/common/STLLoader.cc:105" >> $SUPPRESS
echo "*:gazebo/common/STLLoader.cc:126" >> $SUPPRESS
echo "*:gazebo/common/STLLoader.cc:149" >> $SUPPRESS
echo "*:gazebo/common/Plugin.hh:161" >> $SUPPRESS
echo "*:gazebo/common/Plugin.hh:132" >> $SUPPRESS
echo "*:examples/plugins/custom_messages/custom_messages.cc:22" >> $SUPPRESS
# Not defined FREEIMAGE_COLORORDER
echo "*:gazebo/common/Image.cc:1" >> $SUPPRESS

#cppcheck
CPPCHECK_BASE="cppcheck -q --suppressions-list=$SUPPRESS"
CPPCHECK_INCLUDES="-I gazebo/rendering/skyx/include -I . -I $builddir"\
" -I $builddir/gazebo/msgs -I deps -I deps/opende/include -I test"
CPPCHECK_RULES="--rule-file=./tools/cppcheck_rules/issue_581.rule"
CPPCHECK_CMD1A="-j 4 --enable=style,performance,portability,information"
CPPCHECK_CMD1B="$CPPCHECK_RULES $CPPCHECK_FILES"
CPPCHECK_CMD1="$CPPCHECK_CMD1A $CPPCHECK_CMD1B"
# This command used to be part of the script but was removed since our API
# provides many functions that Gazebo does not use internally
CPPCHECK_CMD2="--enable=unusedFunction $CPPCHECK_FILES"
CPPCHECK_CMD3="-j 4 --enable=missingInclude $CPPCHECK_FILES"\
" $CPPCHECK_INCLUDES --check-config"
if [ $xmlout -eq 1 ]; then
  # Performance, style, portability, and information
  ($CPPCHECK_BASE --xml $CPPCHECK_CMD1) 2> $xmldir/cppcheck.xml

  # Check the configuration
  ($CPPCHECK_BASE --xml $CPPCHECK_CMD3) 2> $xmldir/cppcheck-configuration.xml
elif [ $QUICK_CHECK -eq 1 ]; then
  for f in $CHECK_FILES; do
    prefix=`basename $f | sed -e 's@\..*$@@'`
    ext=`echo $f | sed -e 's@^.*\.@@'`
    tmp2="$QUICK_TMP"."$ext"
    tmp2base=`basename "$QUICK_TMP"`
    hg cat -r $QUICK_SOURCE $hg_root/$f > $tmp2

    if test $ext = "cc"; then
      $CPPCHECK_BASE $CPPCHECK_CMD1A $CPPCHECK_RULES $tmp2 2>&1 \
        | sed -e "s@$tmp2@$f@g" \
        | grep -v 'use --check-config for details'
    fi

    python $hg_root/tools/cpplint.py $tmp2 2>&1 \
      | sed -e "s@$tmp2@$f@g" -e "s@$tmp2base@$prefix@g" \
      | grep -v 'Total errors found: 0'

    rm $tmp2
  done
  rm $QUICK_TMP
else
  # Performance, style, portability, and information
  $CPPCHECK_BASE $CPPCHECK_CMD1 2>&1

  # Check the configuration
  $CPPCHECK_BASE $CPPCHECK_CMD3 2>&1
fi

# cpplint
if [ $xmlout -eq 1 ]; then
  (echo $CPPLINT_FILES | xargs python tools/cpplint.py 2>&1) \
    | python tools/cpplint_to_cppcheckxml.py 2> $xmldir/cpplint.xml
elif [ $QUICK_CHECK -eq 0 ]; then
  echo $CPPLINT_FILES | xargs python tools/cpplint.py 2>&1
fi

# msg_check.py
if [ $xmlout -eq 1 ]; then
  ./tools/msg_check.py xml 2> $xmldir/msg_check.xml
else
  ./tools/msg_check.py 2>&1
fi
