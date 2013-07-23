#!/bin/bash
cat <<END
gzserver -- Gazebo server daemon
================================

## SYNOPSIS

 $(gzserver -h 2>&1 | grep Usage | sed -e 's@^Usage: @@')

## OPTIONS
$(gzserver -h 2>&1 | grep '^ *\-' | sed -e 's@^ *@*  @' -e 's@ *\([A-Z]\)@\n   \1@')

## SEE ALSO
Full user guide can be found at: http://gazebosim.org/user_guide/ 

## AUTHOR
  Open Source Robotics Foundation

## COPYRIGHT
  Copyright 2013 Open Source Robotics Foundation

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
END

