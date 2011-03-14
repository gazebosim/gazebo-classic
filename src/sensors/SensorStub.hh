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
  public: virtual ~SensorStub();

  /// \brief Load the camera using parameter from an XMLConfig node
  /// \param node The XMLConfig node
  protected: virtual void LoadChild( XMLConfigNode *node );

  /// \brief Initialize the camera
  protected: virtual void InitChild();

  /// \brief Update the sensor information
  protected: virtual void UpdateChild();

  /// Finalize the camera
  protected: virtual void FiniChild();

};

/// \}
/// \}
}
#endif

