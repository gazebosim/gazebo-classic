/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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

#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include "gazebo/test/ServerFixture.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/sensors.hh"
#include "gazebo/common/common.hh"

using namespace gazebo;

class SpawnModels : public ServerFixture,
                    public testing::WithParamInterface<const char*>
{
};

/////////////////////////////////////////////////
TEST_P(SpawnModels, WirelessTransmitters)
{
  double gain = 2.6;
  double power = 14.5;

  Load("worlds/empty.world", true, GetParam());

  for (int i = 0; i < 10; ++i)
  {
    for (int j = 0; j < 10; ++j)
    {
      std::string modelName = "tx" +
         boost::lexical_cast<std::string>(i) +
         boost::lexical_cast<std::string>(j);
      std::string sensorName = "WirelessTransmitter" +
         boost::lexical_cast<std::string>(i) +
         boost::lexical_cast<std::string>(j);

      SpawnWirelessTransmitterSensor(modelName, sensorName,
          math::Vector3(i, j, 0.25), math::Vector3(0, 0, 0),
          "osrf", 2450.0, power, gain);

      sensors::WirelessTransmitterPtr tx =
        std::static_pointer_cast<sensors::WirelessTransmitter>(
          sensors::SensorManager::Instance()->GetSensor(sensorName));

      EXPECT_TRUE(this->HasEntity(modelName));
      EXPECT_TRUE(tx != NULL);
    }
  }
}

/////////////////////////////////////////////////
INSTANTIATE_TEST_CASE_P(TestTransceiverODE, SpawnModels,
    ::testing::Values("ode"));

/////////////////////////////////////////////////
#ifdef HAVE_BULLET
INSTANTIATE_TEST_CASE_P(TestTransceiverBullet, SpawnModels,
    ::testing::Values("bullet"));
#endif  // HAVE_BULLET

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
