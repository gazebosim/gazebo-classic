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

# Create working directory
cd 
rm -rf /tmp/gazebo_build
mkdir /tmp/gazebo_build

# Clone
hg clone https://bitbucket.org/osrf/gazebo /tmp/gazebo_build

start_time=`eval date +%s`

# Process each branch from the command line
for branch in $branches
do
  # Get the correct branch
  cd /tmp/gazebo_build
  hg up $branch

  # Build
  rm -rf build
  mkdir build
  cd build
  cmake ../
  make -j4

  echo "Branch: $branch" >> $logfile
  echo "==================================================" >> $logfile
  echo "Test Results" >> $logfile
  # Run make test many times, only capture failures
  for i in {1..100}
  do
    cd /tmp/gazebo_build/build
    make test | grep "***" >> $logfile 
  done

  echo "Code Check Results" >> $logfile
  # Run code checker
  cd /tmp/gazebo_build/
  sh tools/code_check.sh >> $logfile
done
end_time=`eval date +%s`
duration=`expr $end_time - $start_time`
min=`expr $duration / 60`
sec=`expr $duration - $min \* 60`
echo "Duration: $min min $sec sec" >> $logfile

# Cleanup
cd
rm -rf /tmp/gazebo_build
