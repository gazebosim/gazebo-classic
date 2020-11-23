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

#ifndef _GAZEBO_MODEL_MAKER_TEST_HH_
#define _GAZEBO_MODEL_MAKER_TEST_HH_

#include "gazebo/gui/QTestFixture.hh"

/// \brief A test class for the ModelMaker class.
class ModelMaker_TEST : public QTestFixture
{
  Q_OBJECT

  /// \brief Test creating a simple shape.
  private slots: void SimpleShape();

  /// \brief Test creating a model from a file.
  private slots: void FromFile();

  /// \brief Test creating a model from a file with frame semantics.
  private slots: void FromFileWithFrameSemantics();

  /// \brief Test creating a nested model from a file
  private slots: void FromNestedModelFile();

  /// \brief Test creating a model by copying another model.
  private slots: void FromModel();

  /// \brief Test creating a nested model by copying another nested model.
  private slots: void FromNestedModel();

  /// \brief Test creating a model with spaces in the file name and entity
  /// names.
  private slots: void FromModelWithSpaces();

  /// \brief Test creating a nested model with frame semantics (SDFormat 1.7)
  /// from a file
  private slots: void FromNestedModelFileWithFrameSemantics();
};

#endif
