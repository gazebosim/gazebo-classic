#!/bin/bash

# Get the list of branches to check. Use default if no branch specified.
if [ "$#" -ne "0" ]
then
  branches="$@"
else
  branches="default"
fi

# Create a logfile based on the current time
timestamp=`eval date +%d_%m_%Y_%R:%S`
logfile="/tmp/gazebo_test-$timestamp.txt"
logfileVerbose="/tmp/gazebo_test-$timestamp-verbose.txt"
logfileRaw=/tmp/gazebo_build/raw.log

# Create working directory
cd 
rm -rf /tmp/gazebo_build
mkdir /tmp/gazebo_build

# Clone
hg clone https://bitbucket.org/osrf/gazebo /tmp/gazebo_build/source

start_time=`eval date +%s`

PATH=/tmp/gazebo_build/install/bin:$PATH
LD_LIBRARY_PATH=/tmp/gazebo_build/install/lib:$LD_LIBRARY_PATH

# Process each branch from the command line
for branch in $branches
do
  # Get the correct branch
  cd /tmp/gazebo_build/source
  hg up $branch

  # Build
  rm -rf build
  mkdir build
  cd build
  cmake -DCMAKE_INSTALL_PREFIX=/tmp/gazebo_build/install ../
  make -j4 install
  . /tmp/gazebo_build/install/share/gazebo/setup.sh

  echo "Branch: $branch" >> $logfile
  echo "==================================================" >> $logfile
  echo "Test Results" >> $logfile
  # Run make test many times, only capture failures
  for i in {1..10}
  do
    cd /tmp/gazebo_build/source/build

    # make test with verbose output
    make test ARGS="-VV" &> $logfileRaw
    grep '^ *[0-9]*/[0-9]* .*\*\*\*' $logfileRaw >> $logfile

    # for each failed test
    for f in `grep '^ *[0-9]*/[0-9]* .*\*\*\*' $logfileRaw | \
      sed -e 's@^ *\([0-9]*\)/.*@\1@'`
    do
      # output some brief info
      echo Try $i of 10, failed test $f >> $logfileVerbose
      # then send the raw output of both the test and its companion test_ran
      # to the logfile for perusal
      grep '^ *'`echo "(($f-1)/2)*2+1" | bc`':' $logfileRaw >> $logfileVerbose
      grep '^ *'`echo "(($f-1)/2)*2+2" | bc`':' $logfileRaw >> $logfileVerbose
    done
  done

  echo "Code Check Results" >> $logfile
  # Run code checker
  cd /tmp/gazebo_build/source
  sh tools/code_check.sh >> $logfile
done

end_time=`eval date +%s`
duration=`expr $end_time - $start_time`
hour=`expr $duration / 3600`
min=`expr $(( $duration - $hour * 3600 )) / 60`
sec=`expr $duration - $hour \* 3600 - $min \* 60`
echo "Duration: $hour hr $min min $sec sec" >> $logfile

# Cleanup
cd
rm -rf /tmp/gazebo_build
