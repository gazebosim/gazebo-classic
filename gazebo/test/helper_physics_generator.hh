/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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

#ifndef _HELPER_PHYSICS_GENERATOR_HH_
#define _HELPER_PHYSICS_GENERATOR_HH_

#include "gazebo/gazebo_config.h"

#define BULLET_SUPPORT

#ifdef HAVE_BULLET
# undef BULLET_SUPPORT
# define BULLET_SUPPORT , "bullet"
#endif

#define SIMBODY_SUPPORT
#define DART_SUPPORT

#ifdef HAVE_SIMBODY
# undef SIMBODY_SUPPORT
# define SIMBODY_SUPPORT , "simbody"
#endif
#ifdef HAVE_DART
# undef DART_SUPPORT
# define DART_SUPPORT , "dart"
#endif

/// \brief Helper macro to instantiate gtest for different physics engines
#define PHYSICS_ENGINE_VALUES ::testing::Values("ode" \
  BULLET_SUPPORT \
  SIMBODY_SUPPORT \
  DART_SUPPORT \
  )

#endif
