# Detect folder containing this script
_gazebo_current_prefix="$(builtin cd "`dirname "${BASH_SOURCE[0]}"`" > /dev/null && pwd)"
if [ ! -f ${_gazebo_current_prefix}/setup.sh ]; then
  echo This script requires a setup.sh script in the same folder.
  unset _gazebo_current_prefix
  return 1
fi

# Set GAZEBO_INSTALL_PREFIX relative to current folder and source setup.sh
GAZEBO_INSTALL_PREFIX=${_gazebo_current_prefix}/../..
. ${_gazebo_current_prefix}/setup.sh

unset GAZEBO_INSTALL_PREFIX
unset _gazebo_current_prefix
