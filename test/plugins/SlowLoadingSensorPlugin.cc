/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include "SlowLoadingSensorPlugin.hh"

namespace gazebo
{
  /// \brief Private data class for SlowLoadingSensorPluginPrivate
  class SlowLoadingSensorPluginPrivate
  {
  };
}

using namespace gazebo;

GZ_REGISTER_SENSOR_PLUGIN(SlowLoadingSensorPlugin)

/////////////////////////////////////////////////
SlowLoadingSensorPlugin::SlowLoadingSensorPlugin()
  : dataPtr(new SlowLoadingSensorPluginPrivate)
{
}

/////////////////////////////////////////////////
SlowLoadingSensorPlugin::~SlowLoadingSensorPlugin()
{
}

/////////////////////////////////////////////////
void SlowLoadingSensorPlugin::Load(sensors::SensorPtr /*_sensor*/,
                                   sdf::ElementPtr _sdf)
{
  if (!_sdf->HasElement("load_seconds"))
  {
    gzerr << "No model <load_seconds/> detected. No wait will occur."
          << std::endl;
    return;
  }

  double loadSeconds = _sdf->Get<double>("load_seconds");

  gzmsg << "SlowLoadingSensorPlugin: sleeping for " << loadSeconds << " seconds"
        << std::endl;
  common::Time::Sleep(common::Time(loadSeconds));
}
