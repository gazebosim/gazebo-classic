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

#ifndef _GAZEBO_MODEL_EDITOR_TEST_HH_
#define _GAZEBO_MODEL_EDITOR_TEST_HH_

#include "gazebo/gui/QTestFixture.hh"

/// \brief A test class for the model editor
class ModelEditorTest : public QTestFixture
{
  Q_OBJECT

  /// \brief Test editing and saving existing model
  private slots: void EditModel();

  /// \brief Test pose of model links and joints before and after saving
  private slots: void SaveModelPose();

  /// \brief Test pose of joint hotspot visual when parent or child link is
  /// changed via the joint inspector
  private slots: void JointInspectorUpdate();
};

#endif
