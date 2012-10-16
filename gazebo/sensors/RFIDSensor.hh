/* Copyright (C) 2012
 *     Jonas Mellin & Zakiruz Zaman
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
 */
/* Desc: Gazebo RFID Sensor
 * Author: Jonas Mellin & Zakiruz Zaman
 * Date: 6th December 2011
 */

#ifndef _RFIDSENSOR_HH_
#define _RFIDSENSOR_HH_

#include <vector>
#include <string>

#include "transport/TransportTypes.hh"

#include "math/Angle.hh"
#include "math/Pose.hh"

#include "sensors/Sensor.hh"
#include "sensors/RFIDTagManager.hh"

namespace gazebo
{
  /// \ingroup gazebo_sensors
  /// \brief Sensors namespace
  namespace sensors
  {
    /// \addtogroup gazebo_sensors
    /// \{
    /// \
    /// \class RFIDSensor RFIDSensor.hh sensors/sensors.hh
    /// \brief Sensor class for RFID type of sensor
    class RFIDSensor: public Sensor
    {
      /// \brief  Constructor
      public: RFIDSensor();

      /// \brief  Destructor
      public: virtual ~RFIDSensor();

      /// \brief Load the sensor with SDF parameters
      /// \param[in] _sdf SDF Sensor parameters
      /// \param[in] _worldName Name of world to load from
      public: virtual void Load(const std::string &_worldName,
                                sdf::ElementPtr _sdf);

      /// \brief Load the sensor with default parameters
      /// \param[in] _worldName Name of world to load from
      public: virtual void Load(const std::string &_worldName);

      /// \brief  Initialize the sensor
      public: virtual void Init();

      protected: virtual void UpdateImpl(bool _force);

      /// \brief  Finalize the sensor
      public: virtual void Fini();

      /// \brief 
      /// \TODO Nate fill in
      private: void EvaluateTags();

      /// \brief
      /// \TODO Nate fill in
      private: bool CheckTagRange(const math::Pose &_pose);

      /// \brief Checks if ray intersects RFID sensor
      /// \return True if intersects, false if not
      /// \TODO Nate check
      private: bool CheckRayIntersection(const math::Pose &_pose);

      private: physics::EntityPtr entity;

      private: physics::CollisionPtr laserCollision;
      private: physics::RayShapePtr laserShape;

      private: transport::NodePtr node;
      private: transport::PublisherPtr scanPub;

      private: RFIDTagManager *rtm;
    };
    /// \}
  }
}
#endif
