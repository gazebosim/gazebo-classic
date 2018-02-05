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

#ifndef _TEST_INTEGRATION_MODELMANIPULATION_HH_
#define _TEST_INTEGRATION_MODELMANIPULATION_HH_

#include "gazebo/gui/QTestFixture.hh"

/// \brief Integration test for manipulating models through the GUI.
class ModelManipulationTest : public QTestFixture
{
  Q_OBJECT

  /// \brief Verify that model pose stops being updated while it is being
  /// manipulated.
  private slots: void StopProcessingPoseMsgs();

  /// \brief Test switching modes using keyboard shortcuts.
  private slots: void Shortcuts();

  /// \brief Test manipulating models in local and global frames
  private slots: void GlobalLocalFrames();
};

#endif
