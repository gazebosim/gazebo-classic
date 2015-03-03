/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#ifndef _GAZEBO_TEST_ACCURACY_BOXES_HH_
#define _GAZEBO_TEST_ACCURACY_BOXES_HH_

#include <string>
#include "test/ServerFixture.hh"

namespace gazebo
{
  // physics engine
  // dt
  // number of boxes to spawn
  // collision shape on / off
  // nonlinear trajectory on / off
  typedef std::tr1::tuple < const char *
                          , double
                          , int
                          , bool
                          , bool
                          > char1double1int1bool2;
  class BoxesTest : public ServerFixture,
                    public testing::WithParamInterface<char1double1int1bool2>
  {
    /// \brief Test accuracy of unconstrained rigid body motion.
    /// \param[in] _physicsEngine Physics engine to use.
    /// \param[in] _dt Max time step size.
    /// \param[in] _modelCount Number of boxes to spawn.
    /// \param[in] _collision Flag for collision shape on / off.
    /// \param[in] _nonlinear Flag for nonlinear trajectory on / off.
    public: void Boxes(const std::string &_physicsEngine
                     , double _dt
                     , int _modelCount
                     , bool _collision
                     , bool _nonlinear);
  };
}       // namespace gazebo
#endif  // define _GAZEBO_TEST_ACCURACY_BOXES_HH_
