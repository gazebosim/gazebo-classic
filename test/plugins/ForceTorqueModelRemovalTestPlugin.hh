/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

#ifndef _GAZEBO_FORCETORQUE_MODEL_REMOVAL_TEST_PLUGIN_HH_
#define _GAZEBO_FORCETORQUE_MODEL_REMOVAL_TEST_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <string>

namespace gazebo
{
  namespace sensors {
    class ForceTorqueSensor;
  }

  class GAZEBO_VISIBLE ForceTorqueModelRemovalTestPlugin : public SensorPlugin
  {
    public: ForceTorqueModelRemovalTestPlugin();
    public: virtual ~ForceTorqueModelRemovalTestPlugin();

    public: virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);
    public: void onUpdate(const gazebo::common::UpdateInfo & /*_info*/);

    private: virtual void Init();

    private: sensors::ForceTorqueSensor* parentSensor;

    private: std::string sensorName;
    private: double forcetorque_data[6];

    private: gazebo::event::ConnectionPtr updateConnection;
  };
}

#endif
