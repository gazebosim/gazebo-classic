/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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

#ifndef GAZEBO_GUI_MODELMANIPULATOR_TEST_HH_
#define GAZEBO_GUI_MODELMANIPULATOR_TEST_HH_

#include "gazebo/gui/QTestFixture.hh"

/// \brief A test class for the model manipulator tool.
class ModelManipulator_TEST : public QTestFixture
{
  Q_OBJECT

  /// \brief Test attaching to visual
  private slots: void Attach();

  /// \brief Test Transparency of a visual after translating.
  private slots: void Transparency();
};

#endif
