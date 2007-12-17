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
/* Desc: Stubbed out sensor
 * Author: Nate Koenig
 * Date: 05 Aug 2007
 * SVN: $Id$
 */

#ifndef SENSORSTUB_HH
#define SENSORSTUB_HH

#include "Sensor.hh"

namespace gazebo
{
/// \addtogroup gazebo_sensor
/// \brief Stubbed out sensor
/// \{
/// \defgroup gazebo_sensor_stub Sensor Stub
/// \brief Stubbed out sensor
// \{


/// \brief Stubbed out  sensor
///
/// Copy this sensor to create your own
class SensorStub : public Sensor
{
  /// \brief Constructor
  public: SensorStub(Body *body);

  /// \brief Destructor
  public: virtual ~CameraSensor();

  /// \brief Load the camera using parameter from an XMLConfig node
  /// \param node The XMLConfig node
  protected: virtual void LoadChild( XMLConfigNode *node );

  /// \brief Initialize the camera
  protected: virtual void InitChild();

  /// \brief Update the sensor information
  protected: virtual void UpdateChild(UpdateParams &params);

  /// Finalize the camera
  protected: virtual void FiniChild();

};

/// \}
/// \}
}
#endif

