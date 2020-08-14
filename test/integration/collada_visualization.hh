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

#ifndef GAZEBO_TEST_INTEGRATION_COLLADAVISUALIZATION_HH_
#define GAZEBO_TEST_INTEGRATION_COLLADAVISUALIZATION_HH_

#include "gazebo/gui/QTestFixture.hh"

/// \brief A test class for collada visualization.
class ColladaVisualization : public QTestFixture
{
  Q_OBJECT

  /// \brief Test loading a collada mesh that has multiple texture
  /// coordinates
  private slots: void MultipleTextureCoordinates();

  /// \brief Test loading a collada mesh that has multiple inputs
  /// with same offset, see https://github.com/osrf/gazebo/issues/2682
  void AssimpVersion4MultipleInputWithSameOffset();
};

#endif
