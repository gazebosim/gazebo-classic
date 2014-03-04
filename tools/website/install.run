#!/bin/bash

# Description:
# This script installs gazebo onto an Ubuntu system.


codename=`lsb_release -sc`

# Make sure we are running a valid Ubuntu distribution
if [ $codename != "precise" -a $codename != "quantal" -a $codename != "raring" -a $codename != "saucy" ]; then
  echo "This script will only work on Ubuntu precise, quantal, raring, or saucy."
  exit
fi

# Add the OSRF repository
str="sudo sh -c 'echo \"deb http://packages.osrfoundation.org/gazebo/ubuntu ${codename} main\" > /etc/apt/sources.list.d/gazebo_install-latest.list'"
$(eval $str)

# Download the OSRF keys
has_key=`apt-key list | grep "OSRF deb-builder"`

echo "Downloading keys"
if [ -z "$has_key" ]; then
  wget --quiet http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
fi

# Update apt
echo "Retrieving packages"
sudo apt-get update 1>/dev/null
echo "OK"

# Install gazebo
echo "Installing Gazebo"
sudo apt-get install gazebo-current

###
# Cleanup
###

# Remove the apt-repository
sudo rm /etc/apt/sources.list.d/gazebo_install-latest.list

# Remove the key, if it was added.
if [ -z "$has_key" ]; then
  key=`apt-key list | grep -EB1 OSRF | grep pub | cut -d " " -f 4 | cut -d "/" -f 2`
  sudo apt-key del $key 1>/dev/null
fi

# Update apt
sudo apt-get update 1>/dev/null

echo "Complete."
echo "Type gazebo to start the simulator."
