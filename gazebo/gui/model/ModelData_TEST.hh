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

#ifndef _GAZEBO_MODEL_DATA_TEST_HH_
#define _GAZEBO_MODEL_DATA_TEST_HH_

#include "gazebo/gui/QTestFixture.hh"

/// \brief A test class for the ModelData class.
class ModelData_TEST : public QTestFixture
{
  Q_OBJECT

  /// \brief Tests cloning link
  private slots: void Clone();

  /// \brief Tests scaling link
  private slots: void LinkScale();

  /// \brief Tests for computing volume
  private slots: void LinkVolume();

  /// \brief Tests for computing volume of box.
  private slots: void BoxVolume();

  /// \brief Tests for computing volume of cylinder.
  private slots: void CylinderVolume();

  /// \brief Tests for computing volume of sphere.
  private slots: void SphereVolume();

  /// \brief Tests for computing volume of mesh.
  private slots: void MeshVolume();

  /// \brief Tests for computing volume of polyline.
  private slots: void PolylineVolume();

  /// \brief Tests for computing moment of inertia for sphere.
  private slots: void SphereMomentOfInertia();

  /// \brief Tests for computing moment of inertia for cylinder.
  private slots: void CylinderMomentOfInertia();

  /// \brief Tests for computing moment of inertia for box.
  private slots: void BoxMomentOfInertia();

  /// \brief Tests for computing moment of inertia for mesh.
  private slots: void MeshMomentOfInertia();

  /// \brief Tests for computing moment of inertia for polyline.
  private slots: void PolylineMomentOfInertia();
};

#endif
