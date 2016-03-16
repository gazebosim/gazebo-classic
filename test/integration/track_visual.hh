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

#ifndef _GAZEBO_TRACK_VISUAL_TEST_HH_
#define _GAZEBO_TRACK_VISUAL_TEST_HH_

#include <string>
#include "gazebo/gui/QTestFixture.hh"

class QtProperty;
class QtTreePropertyBrowser;

/// \brief Test undo / redo user commands.
class TrackVisualTest : public QTestFixture
{
  Q_OBJECT

  /// \brief Default constructor
  public: TrackVisualTest() = default;

  /// \brief Test tracking visual.
  private slots: void TrackVisual();
};

#endif
