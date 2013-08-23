#!/bin/bash

# Get the list of branches to check. Use default if no branch specified.
if [ "$#" -ne "0" ]
then
  branches="$@"
else
  branches="default"
fi

# Create a logfile based on the current time
timestamp=`eval date +%Y_%m_%d_%R:%S`
logfile="/tmp/gazebo_test-$timestamp.txt"
logfileSummary="/tmp/gazebo_test-$timestamp-summary.txt"
logfileVerbose="/tmp/gazebo_test-$timestamp-verbose.txt"
BUILD_ROOT=/tmp/gazebo_build
logfileRaw=$BUILD_ROOT/raw.log

# Create working directory
cd 
rm -rf $BUILD_ROOT
mkdir $BUILD_ROOT

# Clone
ORIGIN=https://bitbucket.org/osrf/gazebo
if [ `hostname` == t2 ]
then
  hg clone $HOME/osrf/gazebo_readonly $BUILD_ROOT/source
  cd $BUILD_ROOT/source
  hg pull $ORIGIN
elif [ `hostname` == t1000 ]
then
  hg clone $HOME/code/gazebo $BUILD_ROOT/source
  cd $BUILD_ROOT/source
  hg pull $ORIGIN

  # build against homebrew
  export PATH=/usr/local/bin:/usr/local/sbin:$PATH
  export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH
  export PKG_CONFIG_LIBDIR=$PKG_CONFIG_LIBDIR:/usr/lib/pkgconfig:/usr/local/Library/ENV/pkgconfig/10.8
  export PYTHONPATH=/usr/local/lib/python2.7/site-packages:$PYTHONPATH
else
  hg clone $ORIGIN $BUILD_ROOT/source
fi

start_time=`eval date +%s`

export DISPLAY=:0
export PATH=$BUILD_ROOT/install/bin:$PATH
export LD_LIBRARY_PATH=$BUILD_ROOT/install/lib:$LD_LIBRARY_PATH
export PKG_CONFIG_PATH=$BUILD_ROOT/install/lib/pkgconfig:$PKG_CONFIG_PATH
ulimit -c unlimited

# Process each branch from the command line
for branch in $branches
do
  # Get the correct branch
  cd $BUILD_ROOT/source
  echo hg up $branch
  hg up $branch

  # Build
  rm -rf build
  mkdir build
  cd build
  cmake .. \
    -DCMAKE_INSTALL_PREFIX=$BUILD_ROOT/install \
    -DFORCE_GRAPHIC_TESTS_COMPILATION=True
  make -j4 install
  . $BUILD_ROOT/install/share/gazebo/setup.sh

  echo "Branch: $branch" >> $logfile
  echo "==================================================" >> $logfile

  echo "Code Check Results" >> $logfile
  # Run code checker
  cd $BUILD_ROOT/source
  sh tools/code_check.sh >> $logfile

  echo "Test Results" >> $logfile
  # Run make test many times, only capture failures
  for i in {1..100}
  do
    cd $BUILD_ROOT/source/build

    # make test with verbose output
    make test ARGS="-VV" &> $logfileRaw
    grep '^ *[0-9]*/[0-9]* .*\*\*\*' $logfileRaw >> $logfile
    echo make test try $i of 100, \
      `grep '^ *[0-9]*/[0-9]* .*\*\*\*' $logfileRaw | wc -l` \
      tests failed

    # for each failed test
    for f in `grep '^ *[0-9]*/[0-9]* .*\*\*\*' $logfileRaw | \
      sed -e 's@^ *\([0-9]*\)/.*@\1@'`
    do
      # output some brief info
      echo Branch "$branch" try $i of 100, failed test $f >> $logfileVerbose
      # then send the raw output of both the test and its companion test_ran
      # to the logfile for perusal
      grep '^ *'`echo "(($f-1)/2)*2+1" | bc`':' $logfileRaw >> $logfileVerbose
      grep '^ *'`echo "(($f-1)/2)*2+2" | bc`':' $logfileRaw >> $logfileVerbose
    done

    # update summary file
    grep 'Test *#' $logfile  | \
      sed -e 's@[0-9]*\.[0-9]* *sec$@@' -e 's@.*Test@failures:@' | \
      sort | uniq -c | sort -rg > $logfileSummary
  done
done

end_time=`eval date +%s`
duration=`expr $end_time - $start_time`
hour=`expr $duration / 3600`
min=`expr $(( $duration - $hour * 3600 )) / 60`
sec=`expr $duration - $hour \* 3600 - $min \* 60`
echo "Duration: $hour hr $min min $sec sec" >> $logfile

# Cleanup
cd
rm -rf $BUILD_ROOT
