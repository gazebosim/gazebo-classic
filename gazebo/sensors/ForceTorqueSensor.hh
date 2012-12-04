/*
 * Copyright 2012 Open Source Robotics Foundation
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
/* Desc: ForceTorque sensor
 * Author: Nate Koenig
 * Date: 09 Sept. 2008
*/

#ifndef _FORCE_TORQUE_SENSOR_HH_
#define _FORCE_TORQUE_SENSOR_HH_

#include <vector>
#include <map>
#include <list>
#include <string>

#include "gazebo/msgs/msgs.hh"

#include "gazebo/math/Angle.hh"
#include "gazebo/sensors/Sensor.hh"
#include "gazebo/physics/JointWrench.hh"

namespace gazebo
{
  /// \ingroup gazebo_sensors
  /// \brief Sensors namespace
  namespace sensors
  {
    /// \addtogroup gazebo_sensors
    /// \{

    /// \class ForceTorqueSensor ForceTorqueSensor.hh sensors/sensors.hh
    /// \brief ForceTorque sensor. This sensor detects and reports constraint forces between
    ///  objects
    class ForceTorqueSensor: public Sensor
    {
      /// \brief Constructor.
      public: ForceTorqueSensor();

      /// \brief Destructor.
      public: virtual ~ForceTorqueSensor();

      /// \brief Load the sensor with SDF parameters
      /// \param[in] _sdf SDF Sensor parameters
      /// \param[in] _worldName Name of world to load from
      public: virtual void Load(const std::string &_worldName,
                                sdf::ElementPtr _sdf);

      /// \brief Load the sensor with default parameters.
      /// \param[in] _worldName Name of world to load from.
      public: virtual void Load(const std::string &_worldName);

      /// \brief Initialize the sensor.
      public: virtual void Init();

      /// \brief Update the sensor information.
      /// \param[in] _force True if update is forced, false if not.
      protected: virtual void UpdateImpl(bool _force);

      /// \brief Finalize the sensor.
      protected: virtual void Fini();

      // Documentation inherited.
      public: virtual bool IsActive();

      /// \brief Callback for force torque messages from the physics engine.
      private: void OnForceTorques(ConstForceTorquePtr &_msg);

      /// \brief Output force torque information.
      private: transport::PublisherPtr forceTorquesPub;

      /// \brief Subscription to force torque messages from the physics engine
      private: transport::SubscriberPtr forceTorqueSub;

      /// \brief Mutex to protect reads and writes.
      private: boost::mutex mutex;

      /// \brief ForceTorque message used to output sensor data.
      private: msgs::ForceTorque forceTorquesMsg;

      typedef std::list<boost::shared_ptr<msgs::ForceTorque const> > ForceTorqueMsgs_L;
      private: ForceTorqueMsgs_L incomingForceTorques;
    };
    /// \}
  }
}
#endif
