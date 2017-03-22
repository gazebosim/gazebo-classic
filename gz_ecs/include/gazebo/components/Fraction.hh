/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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

#ifndef GAZEBO_GZ_ECS_COMPONENTS_FRACTION_HH_
#define GAZEBO_GZ_ECS_COMPONENTS_FRACTION_HH_

namespace gazebo
{
  namespace components
  {
    /// \brief A fraction that can be divided
    ///
    /// Why a fraction component instead of a numerator and a denominator?
    /// It's becomming clear that the best possible cache performance means
    /// One component per system. Combining the related data into one
    /// component means the cache can be filled with a list of components,
    /// And the system which operates on them (in this case the division system)
    /// can iterate through all those components and do useful work.
    struct Fraction
    {
      float numerator;
      float denominator;
      float test;
    };
  }
}

#endif
