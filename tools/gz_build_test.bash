#!/bin/bash

# Get the list of branches to check. Use default if no branch specified.
if [ "$#" -ne "0" ]
then
  branches="$@"
else
  branches="default"
fi

# Create a logfile based on the current time
PREFIX=/data_fast/scpeters/ws/tmp
timestamp=`eval date +%Y_%m_%d_%R:%S`
logfile="${PREFIX}/gazebo_test-$timestamp.txt"
logfileSummary="${PREFIX}/gazebo_test-$timestamp-summary.txt"
logfileVerbose="${PREFIX}/gazebo_test-$timestamp-verbose.txt"
BUILD_ROOT=${PREFIX}/gazebo_build
logfileRaw=$BUILD_ROOT/raw.log
testCount=18

# Create catkin workspace
cd 
rm -rf $BUILD_ROOT
mkdir -p $BUILD_ROOT/src

# Clone
CATKIN_ORIGIN=https://github.com/ros/catkin.git
GAZEBO_ORIGIN=https://bitbucket.org/osrf/gazebo
#SDFORMAT_ORIGIN=https://bitbucket.org/osrf/sdformat
if [ `hostname` == t2 ]
then
  hg clone $HOME/osrf/gazebo $BUILD_ROOT/src/gazebo
  cd $BUILD_ROOT/src/gazebo
  hg pull $GAZEBO_ORIGIN
#  hg clone $HOME/osrf/sdformat $BUILD_ROOT/src/sdformat
#  cd $BUILD_ROOT/src/sdformat
#  hg pull $SDFORMAT_ORIGIN
  hg up default
else
  hg clone $GAZEBO_ORIGIN $BUILD_ROOT/src/gazebo
#  hg clone $SDFORMAT_ORIGIN $BUILD_ROOT/src/sdformat
fi
git clone $CATKIN_ORIGIN $BUILD_ROOT/src/catkin

# get package.xml files
curl https://bitbucket.org/scpeters/unix-stuff/raw/master/package_xml/package_gazebo.xml > $BUILD_ROOT/src/gazebo/package.xml
#curl https://bitbucket.org/scpeters/unix-stuff/raw/master/package_xml/package_sdformat.xml > $BUILD_ROOT/src/sdformat/package.xml
cd $BUILD_ROOT && catkin init

start_time=`eval date +%s`

export DISPLAY=:0
ulimit -c unlimited

# Process each branch from the command line
for branch in $branches
do
  # Get the correct branch
  cd $BUILD_ROOT/src/gazebo
  echo hg up $branch
  hg up $branch

  # Apply testing patch (usually for more console messages)
  touch $HOME/bin/gz_build_test_scpeters.patch
  patch -p1 < $HOME/bin/gz_build_test_scpeters.patch

  # Build
  catkin clean -a
#    -DGAZEBO_RUN_VALGRIND_TESTS:BOOL=True \
  catkin config --cmake-args \
    -DENABLE_SCREEN_TESTS:BOOL=True \
    -DFORCE_GRAPHIC_TESTS_COMPILATION:BOOL=True
  catkin build
  . $BUILD_ROOT/devel/setup.bash
  . $BUILD_ROOT/devel/share/gazebo/setup.sh

  echo "Branch: $branch" >> $logfile
  echo "hg id: `hg id`" >> $logfile
  echo "==================================================" >> $logfile

  echo "Code Check Results" >> $logfile
  # Run code checker
  cd $BUILD_ROOT/source
  sh tools/code_check.sh >> $logfile

  # Run make test many times, only capture failures

  for i in `seq $testCount`
  do
    echo "Test results try $i of $testCount" >> $logfile
    cd $BUILD_ROOT/build/gazebo

    # make test with verbose output
    make test ARGS="-VV" &> $logfileRaw
    grep '^ *[0-9]*/[0-9]* .*\*\*\*' $logfileRaw >> $logfile
    echo make test try $i of $testCount, \
      `grep '^ *[0-9]*/[0-9]* .*\*\*\*' $logfileRaw | wc -l` \
      tests failed

    # for each failed test
    for f in `grep '^ *[0-9]*/[0-9]* .*\*\*\*' $logfileRaw | \
      sed -e 's@^ *\([0-9]*\)/.*@\1@'`
    do
      # output some brief info
      echo Branch "$branch" try $i of $testCount, failed test $f >> $logfileVerbose
      # then send the raw output of both the test and its companion test_ran
      # to the logfile for perusal
      grep '^ *'`echo "(($f-1)/2)*2+1" | bc`':' $logfileRaw | grep -v YANKING >> $logfileVerbose
      grep '^ *'`echo "(($f-1)/2)*2+2" | bc`':' $logfileRaw | grep -v YANKING >> $logfileVerbose
    done

    # update summary file
    grep 'Test *#' $logfile  | \
      sed -e 's@ *[0-9]*\.[0-9]* *sec$@@' -e 's@.*Test@failures:@' | \
      sort | uniq -c | sort -rg > $logfileSummary
  done
done

end_time=`eval date +%s`
duration=`expr $end_time - $start_time`
hour=`expr $duration / 3600`
min=`expr $(( $duration - $hour * 3600 )) / 60`
sec=`expr $duration - $hour \* 3600 - $min \* 60`
echo "Duration: $hour hr $min min $sec sec" >> $logfile
cp $logfileRaw $PREFIX

# Cleanup
cd
rm -rf $BUILD_ROOT
