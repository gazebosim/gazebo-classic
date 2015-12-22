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

#ifndef _GAZEBO_TOP_TOOLBAR_TEST_HH_
#define _GAZEBO_TOP_TOOLBAR_TEST_HH_

#include "gazebo/gui/QTestFixture.hh"

/// \brief A test class for the TopToolbar widget.
class TopToolbar_TEST : public QTestFixture
{
  Q_OBJECT

  /// \brief Test different window modes.
  private slots: void WindowModes();

  /// \brief Test inserting actions, separators and widgets.
  private slots: void Insert();

  /// \brief Test adding actions, separators and widgets.
  private slots: void Add();
};

#endif
