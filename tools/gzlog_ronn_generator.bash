#!/bin/bash
cat <<END
gzlog -- Tool to instrospect Gazebo log files
=============================================

## SYNOPSIS

 $(gzlog -h 2>&1 | grep Usage | sed -e 's@^Usage: @@')

## OPTIONS
$(gzlog -h 2>&1 | grep '^ *\-' | sed -e 's@^ *@*  @' -e 's@ *\([A-Z]\)@\n   \1@')

## SEE ALSO
Example and more information about gazebo gzfactory and other command
line tools can be found at:
http://gazebosim.org/user_guide/started__commandlinetools.html

## AUTHOR
  Open Source Robotics Foundation

## COPYRIGHT
  Copyright 2013 Open Source Robotics Foundation

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
END

