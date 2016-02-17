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

#ifndef _GAZEBO_MODEL_PLUGIN_INSPECTOR_TEST_HH_
#define _GAZEBO_MODEL_PLUGIN_INSPECTOR_TEST_HH_

#include "gazebo/gui/QTestFixture.hh"

/// \brief A test class for the model plugin inspector.
class ModelPluginInspector_TEST : public QTestFixture
{
  Q_OBJECT

  /// \brief Test the inspector's buttons.
  private slots: void Buttons();

  /// \brief Test updating the inspector.
  private slots: void Update();

  /// \brief Test pressing remove button.
  private slots: void RemoveButton();

  /// \brief Tests setting the inspector to be read-only.
  private slots: void ReadOnly();

  /// \brief Tests clearing the inspector.
  private slots: void Clear();

  /// \brief Tests getting plugin msg data.
  private slots: void GetData();
};

#endif
