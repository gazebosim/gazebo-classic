/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
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
#ifndef GAZEBO_GUI_LASERVISUALIZATION_TEST_HH_
#define GAZEBO_GUI_LASERVISUALIZATION_TEST_HH_

#include "gazebo/gui/QTestFixture.hh"

/// \brief A test class for laser visualization.
class LaserVisualization_TEST : public QTestFixture
{
  Q_OBJECT

  /// \brief Test that laser visualization draws lines correctly.
  private slots: void Lines();

  /// \brief Laser hit visualization.
  private slots: void Hit();

  /// \brief Laser no-hit visualization.
  private slots: void Nohit();

  /// \brief Test laser deadzone visualization.
  private slots: void Deadzone();
};

#endif
