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
timestamp=`eval date +%Y_%m_%d_%H_%M_%S`
logfile="${PREFIX}/gazebo_test-$timestamp.txt"
logfileSummary="${PREFIX}/gazebo_test-$timestamp-summary.txt"
junit_prefix="${PREFIX}/gazebo_test-$timestamp"
BUILD_ROOT=${PREFIX}/gazebo_build
logfileRaw=$BUILD_ROOT/raw.log
testCount=15

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
  cd $BUILD_ROOT/src/gazebo
  sh tools/code_check.sh >> $logfile

  # Run make test many times, only capture failures

  for i in `seq $testCount`
  do
    echo "Test results try $i of $testCount" >> $logfile
    cd $BUILD_ROOT/build/gazebo

    # make test with verbose output
    rm -rf test_results
    mkdir test_results
    make test ARGS="-VV" &> $logfileRaw
    grep '^ *[0-9]*/[0-9]* .*\*\*\*' $logfileRaw >> $logfile
    echo make test try $i of $testCount, \
      `grep '^ *[0-9]*/[0-9]* .*\*\*\*' $logfileRaw | wc -l` \
      tests failed

    # update summary file
    grep 'Test *#' $logfile  | \
      sed -e 's@ *[0-9]*\.[0-9]* *sec$@@' -e 's@.*Test@failures:@' | \
      sort | uniq -c | sort -rg > $logfileSummary

    junit_prefix_try="${junit_prefix}-try-$i"
    tar cfz ${junit_prefix_try}-test_results.tar.gz test_results

    # for each failed test
    for f in `grep '^ *[0-9]*/[0-9]* .*\*\*\*' $logfileRaw | \
      sed -e 's@^ *\([0-9]*\)/.*@\1@'`
    do
      junit_prefix_test_try="${junit_prefix}-test-$f-try-$i"
      # then send the raw output of both the test and its companion test_ran
      # to *-console.txt
      grep '^ *'$(echo "(($f-1)/2)*2+1" | bc)':' $logfileRaw >  ${junit_prefix_test_try}-console.txt
      grep '^ *'$(echo "(($f-1)/2)*2+2" | bc)':' $logfileRaw >> ${junit_prefix_test_try}-console.txt
      # save junit of failed tests
      JUNIT=$(grep '^ *'$(echo "(($f-1)/2)*2+1" | bc)': Test command' $logfileRaw | \
              tr -d '"' | \
              grep xml$ | \
              sed -e 's@.*-xml -o @@' \
                  -e 's@.*--gtest_output=xml:@@' \
                  -e 's@.*check_test_ran.py @@' \
      )
      cp ${JUNIT} ${junit_prefix_test_try}-$(basename ${JUNIT})
    done

    # for each coredump
    for c in $(find . | grep '/core$')
    do
      CORE_EXECUTABLE_PATH=$(readelf -n $c | grep -A2 '^ *Start *End *Page Offset' | tail -1)
      CORE_EXECUTABLE_NAME=$(basename ${CORE_EXECUTABLE_PATH})
      CORE_TEST_NUMBER=$(grep "${CORE_EXECUTABLE_NAME}\.xml" $logfileRaw | head -1 | sed -e 's@:.*@@')
      if [[ "${CORE_EXECUTABLE_NAME}" == "UNIT_Exception_TEST" || \
            "${CORE_EXECUTABLE_NAME}" == "UNIT_ModelSnap_TEST" ]]; then
        rm $c
        continue
      elif [[ -z "${CORE_TEST_NUMBER}" ]]; then
        CORE_TEST_NUMBER=${CORE_EXECUTABLE_NAME}
      fi
      CORE_BT_LOG="${junit_prefix}-test-$CORE_TEST_NUMBER-try-$i-backtrace.txt"
      echo '# CORE_TEST_NUMBER='"${CORE_TEST_NUMBER}" >> $CORE_BT_LOG
      echo '# CORE_EXECUTABLE_NAME='"${CORE_EXECUTABLE_NAME}" >> $CORE_BT_LOG
      echo '# CORE_EXECUTABLE_PATH='"${CORE_EXECUTABLE_PATH}" >> $CORE_BT_LOG
      echo '# ls -l: '"$(ls -l ${c})" >> $CORE_BT_LOG
      gdb $CORE_EXECUTABLE_PATH $c -ex bt -ex 'thread apply all bt' -ex q >> $CORE_BT_LOG
      rm $c
      if grep OnReadHeader $CORE_BT_LOG; then
        mv $CORE_BT_LOG "${junit_prefix}-test-$CORE_TEST_NUMBER-try-$i-OnReadHeader-backtrace.txt"
      elif grep IOManager::Stop $CORE_BT_LOG; then
        mv $CORE_BT_LOG "${junit_prefix}-test-$CORE_TEST_NUMBER-try-$i-IOManager-backtrace.txt"
      elif grep DispatchDiscoveryMsg $CORE_BT_LOG; then
        mv $CORE_BT_LOG "${junit_prefix}-test-$CORE_TEST_NUMBER-try-$i-DispatchDiscoveryMsg-backtrace.txt"
      elif grep World::BuildSceneMsg $CORE_BT_LOG && \
           grep World::RemoveModel $CORE_BT_LOG; then
        mv $CORE_BT_LOG "${junit_prefix}-test-$CORE_TEST_NUMBER-try-$i-RemoveModel_BuildSceneMsg-backtrace.txt"
      elif grep World::BuildSceneMsg $CORE_BT_LOG && \
           grep SimTk::StateImpl::getCacheEntry $CORE_BT_LOG; then
        mv $CORE_BT_LOG "${junit_prefix}-test-$CORE_TEST_NUMBER-try-$i-SimTk_BuildSceneMsg-backtrace.txt"
      elif grep Visual::GetBoundsHelper $CORE_BT_LOG && \
           grep simbody $CORE_BT_LOG; then
        mv $CORE_BT_LOG "${junit_prefix}-test-$CORE_TEST_NUMBER-try-$i-GetBoundsHelper_simbody-backtrace.txt"
      elif grep World::LogWorker $CORE_BT_LOG && \
           grep std::_Rb_tree_increment $CORE_BT_LOG && \
           grep 'WorldState::operator-' $CORE_BT_LOG; then
        mv $CORE_BT_LOG "${junit_prefix}-test-$CORE_TEST_NUMBER-try-$i-LogWorker_WorldState-backtrace.txt"
      fi
    done
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
