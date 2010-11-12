/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003  
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
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

