/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
#ifndef _GAZEBO_RFIDTAG_HH_
#define _GAZEBO_RFIDTAG_HH_

#include <vector>
#include <string>
#include <ignition/math/Pose3.hh>

#include "gazebo/common/CommonTypes.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/sensors/Sensor.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace sensors
  {
    // Forward declare private data class.
    class RFIDTagPrivate;

    /// \addtogroup gazebo_sensors
    /// \{

    /// \class RFIDTag RFIDTag.hh sensors/sensors.hh
    /// \brief RFIDTag to interact with RFIDTagSensors
    class GAZEBO_VISIBLE RFIDTag: public Sensor
    {
      /// \brief Constructor.
      public: RFIDTag();

      /// \brief Destructor.
      public: virtual ~RFIDTag();

      // Documentation inherited
      public: virtual void Load(const std::string & _worldName,
                                sdf::ElementPtr _sdf);

      // Documentation inherited
      public: virtual void Load(const std::string & _worldName);

      // Documentation inherited
      public: virtual void Init();

      // Documentation inherited
      protected: virtual bool UpdateImpl(const bool _force);

      // Documentation inherited
      public: virtual void Fini();

      /// \brief Returns pose of tag in world coordinate.
      /// \return Pose of object.
      public: ignition::math::Pose3d TagPose() const;

      /// \internal
      /// \brief Private data pointer.
      private: std::shared_ptr<RFIDTagPrivate> dataPtr;
    };
    /// \}
  }
}
#endif
