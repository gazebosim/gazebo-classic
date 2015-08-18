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

#ifndef _WIDEANGLECAMERASENSOR_HH_
#define _WIDEANGLECAMERASENSOR_HH_

#include "CameraSensor.hh"


namespace gazebo
{
  namespace sensors
  {
    /// \addtogroup gazebo_sensors Sensors
    /// \{

    /// \class WideAngleCameraSensor WideAngleCameraSensor.hh sensors/sensors.hh
    /// \brief Basic camera sensor
    ///
    /// This sensor is used for simulating cameras with wide angle lens
    class WideAngleCameraSensor : public CameraSensor
    {
      /// \brief Constructor
      public: WideAngleCameraSensor();

      /// \brief Destructor
      public: virtual ~WideAngleCameraSensor();

      // Documentation inherited
      public: void Load(const std::string &_worldName) override;

      // Documentation inherited
      public: void Init() override;

      // Documentation inherited
      protected: void Fini() override;

      // Documentation inherited
      protected: bool UpdateImpl(bool _force) override;

      /// \brief Handle incoming control message
      /// \param[in] _msg Message received from topic
      protected: void OnCtrlMessage(ConstCameraLensCmdPtr &_msg);

      /// \brief Publisher of lens info messages
      protected: transport::PublisherPtr lensPub;

      /// \brief Subscriber to lens control messages
      protected: transport::SubscriberPtr lensSub;

      /// \brief Mutex to lock when receiving or sending lens control/info message
      protected: std::mutex lensCmdMutex;
    };
    /// \}
  }
}

#endif
