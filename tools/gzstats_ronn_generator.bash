#!/bin/bash
cat <<END
gzstats -- This tool displays statistics about a running Gazebo world
=====================================================================

## SYNOPSIS

 $(gzstats -h 2>&1 | grep Usage | sed -e 's@^Usage: @@')

## OPTIONS
$(gzstats -h 2>&1 | grep '^ *\-' | sed -e 's@^ *@*  @' -e 's@ *\([A-Z]\)@\n   \1@')

## SEE ALSO
Example and more information about gazebo gzfactory and other command
line tools can be found at:
http://gazebosim.org/wiki/Tools#World_Statistics

## AUTHOR
  Open Source Robotics Foundation

## COPYRIGHT
  Copyright 2013 Open Source Robotics Foundation

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
END

