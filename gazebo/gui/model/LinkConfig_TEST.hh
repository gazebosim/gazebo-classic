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

#ifndef _GAZEBO_LINKCONFIG_TEST_HH_
#define _GAZEBO_LINKCONFIG_TEST_HH_

#include "gazebo/gui/QTestFixture.hh"

/// \brief A test class for the link config object.
class LinkConfig_TEST : public QTestFixture
{
  Q_OBJECT

  /// \brief Test initialization.
  private slots: void Initialization();

  /// \brief Test link message update
  private slots: void LinkMsgUpdate();

  /// \brief Test pose update
  private slots: void PoseUpdate();

  /// \brief Test mass update
  private slots: void MassUpdate();

  /// \brief Test inertia update
  private slots: void InertiaMatrixUpdate();

  /// \brief Test inertia update
  private slots: void InertialPoseUpdate();
};

#endif
