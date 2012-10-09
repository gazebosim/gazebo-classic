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

#ifndef IMUSENSOR_HH
#define IMUSENSOR_HH

#include <vector>
#include <string>

#include "Sensor.hh"

namespace gazebo
{
  namespace sensors
  {
    /// \TODO This class inherits from Sensor, but looks like it specifically 
    ///       doesn't override any methods, is this intentional? i.e. LoadChild 
    ///       instead of Load, InitChild instead of Init
    /// \class ImuSensor ImuSensor.hh sensors/sensors.hh
    /// \addtogroup gazebo_sensors
    /// \{
    /// \brief An IMU sensor
    class ImuSensor: public Sensor
    {
      /// \brief Constructor
      /// \param body The IMU sensor must be attached to a body.
      public: ImuSensor(Body *_body);

      /// \brief Destructor
      public: virtual ~ImuSensor();

      /// \brief Load the ImuSensor from XMLConfigNode
      /// \param node The XMLConfig node
      protected: virtual void LoadChild(XMLConfigNode *_node);

      /// \brief Save the sensor info in XML format
      /// \param _prefix 
      /// \param _stream
      /// \TODO Nate fill in
      protected: virtual void SaveChild(std::string &_prefix,
                                        std::ostream &_stream);

      /// \brief Initialize the ray
      protected: virtual void InitChild();

      /// \brief Update sensed values
      protected: virtual void UpdateChild();

      /// \brief Finalize the ray
      protected: virtual void FiniChild();

      /// \brief Get velocity from sensor
      /// \return velocity data stored in Pose
      /// \TODO Nate check
      public: Pose GetVelocity();

      private: Pose prevPose;
      private: Pose imuVel;
    };
    /// \}
  }
}
#endif

