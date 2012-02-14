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
#include "ServerFixture.hh"

using namespace gazebo;
class PR2Test : public ServerFixture
{
};

TEST_F(PR2Test, Load)
{
  Load("worlds/empty.world");
  SpawnModel("models/pr2.model");

  int i;
  for (i = 0; i < 20 && !this->HasEntity("pr2"); ++i)
    common::Time::MSleep(100);
  EXPECT_LT(i, 20);

  sensors::SensorPtr sensor =
    sensors::get_sensor("narrow_stereo_gazebo_l_stereo_camera_sensor");
  EXPECT_TRUE(sensor);

  sensors::CameraSensorPtr camSensor =
    boost::shared_dynamic_cast<sensors::CameraSensor>(sensor);
  EXPECT_TRUE(camSensor);

  /*while (!camSensor->SaveFrame("/tmp/frame_10.jpg"))
    usleep(100000);

  for (int i = 11; i < 100; i++)
  {
    std::ostringstream filename;
    filename << "/tmp/frame_" << i << ".jpg";
    camSensor->SaveFrame(filename.str());
    usleep(100000);
  }
  */
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
