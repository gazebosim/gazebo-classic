/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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

#ifndef _MODELALIGN_TEST_HH_
#define _MODELALIGN_TEST_HH_

#include "gazebo/gui/QTestFixture.hh"

/// \brief A test class for the model align tool.
class ModelAlign_TEST : public QTestFixture
{
  Q_OBJECT
  
  /// \brief Test aligning models at min x.
  private slots: void AlignXMin();

  /// \brief Test centering models in the x axis.
  private slots: void AlignXCenter();

  /// \brief Test aligning models at max x.
  private slots: void AlignXMax();

  /// \brief Test aligning models at min y.
  private slots: void AlignYMin();

  /// \brief Test centering models in the y axis.
  private slots: void AlignYCenter();

  /// \brief Test aligning models at max y.
  private slots: void AlignYMax();

  /// \brief Test aligning models at min z.
  private slots: void AlignZMin();

  /// \brief Test centering models in the z axis.
  private slots: void AlignZCenter();

  /// \brief Test aligning models at max z.
  private slots: void AlignZMax();

  /// \brief Test aligning models with non unit scale.
  private slots: void AlignScale();
};

#endif
