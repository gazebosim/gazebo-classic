/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
/* Desc: A persepective X11 OpenGL Camera Sensor
 * Author: Nate Koenig
 * Date: 15 July 2003
 * CVS: $Id$
 */

#ifndef MONOCAMERASENSOR_HH
#define MONOCAMERASENSOR_HH

#include "Camera.hh"
#include "Sensor.hh"

namespace gazebo
{
/// \addtogroup gazebo_sensor
/// \brief Basic camera sensor
/// \{
/// \defgroup gazebo_camera Camera
/// \brief Basic camera sensor
// \{


/// \brief Basic camera sensor
///
/// This sensor is used for simulating standard monocular cameras; is
/// is used by both camera models (e.g., SonyVID30) and user interface
/// models (e.g., ObserverCam).
class MonoCameraSensor : public Sensor
{
  /// \brief Constructor
  public: MonoCameraSensor(Body *body);

  /// \brief Destructor
  public: virtual ~MonoCameraSensor();

  /// \brief Get the camera
  public: Camera *GetCamera();

  /// \brief Load the camera using parameter from an XMLConfig node
  /// \param node The XMLConfig node
  protected: virtual void LoadChild( XMLConfigNode *node );

  /// \brief Save the sensor info in XML format
  protected: virtual void SaveChild(std::string &prefix, std::ostream &stream);

  /// \brief Initialize the camera
  protected: virtual void InitChild();

  /// \brief Update the sensor information
  protected: virtual void UpdateChild();

  /// Finalize the camera
  protected: virtual void FiniChild();

  /// \brief Set whether the sensor is active or not
  public: virtual void SetActive(bool value);

  public: virtual std::string GetName() const { return Sensor::GetName(); }

  private: void Render();

  private: Camera *camera;

  protected: std::string ogreTextureName;
};

/// \}
/// \}
}
#endif

