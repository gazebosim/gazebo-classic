/* Copyright (C)
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
/* Desc: RFID Tag
 * Author: Jonas Mellin & Zakiruz Zaman 
 * Date: 6th December 2011
 */

#ifndef RFIDTAG_HH
#define RFIDTAG_HH

#include <vector>
#include <string>

#include "transport/TransportTypes.hh"
#include "sensors/Sensor.hh"
#include "math/gzmath.hh"
#include "physics/physics.hh"

namespace gazebo
{
  namespace sensors
  {
    /// \addtogroup gazebo_sensors
    /// \{

    /// \brief RFID tag sensor
    class RFIDTag: public Sensor
    {
      /// \brief  Constructor
      public: RFIDTag();

      /// \brief  Destructor
      public: virtual ~RFIDTag();

      /// \brief Load the sensor with SDF parameters
      /// \param _sdf SDF Sensor parameteres
      public: virtual void Load(const std::string & _worldName,
                                sdf::ElementPtr &_sdf);

      /// \brief Load the sensor with default parameters
      public: virtual void Load(const std::string & _worldName);

      /// \brief  Initialize the sensor
      public: virtual void Init();

      protected: virtual void UpdateImpl(bool _force);

      /// \brief  Finalize the sensor
      public: virtual void Fini();

      /// \brief  returns pose of tag in world coordinate
      public: math::Pose GetTagPose() {return entity->GetWorldPose();}

      private: physics::EntityPtr entity;
      private: transport::NodePtr node;
      private: transport::PublisherPtr scanPub;
    };
    /// \}
  }
}
#endif
