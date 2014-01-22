#!/bin/bash
lower=`echo $1 | tr '[:upper:]' '[:lower:]'`
guard=_GAZEBO_`echo $1 | tr '[:lower:]' '[:upper:]'`_HH_
cat <<END
/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#ifndef $guard
#define $guard

// Deprecated header file for case-sensitive filesystems
#warning The gazebo/$lower/$1.hh header file is deprecated \\
  as of gazebo 1.9 and will be removed in the next release. \\
  Please include gazebo/$lower/$1Iface.hh instead.
#include "gazebo/$lower/$1Iface.hh"

#endif

END

