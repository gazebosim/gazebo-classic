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
#include "ServerFixture.hh"

using namespace gazebo;
class SpeedTest : public ServerFixture
{
};


TEST_F(SpeedTest, PR2World)
{
  Load("worlds/empty.world");
  double emptySpeed;
  while ((emptySpeed = GetPercentRealTime()) == 0)
    common::Time::MSleep(100);
  common::Time::MSleep(2000);
  emptySpeed = GetPercentRealTime();

  // Load the pr2into the world
  SpawnModel("model://pr2");
  common::Time::MSleep(2000);
  double loadedSpeed = GetPercentRealTime();

  double speedRatio = loadedSpeed / emptySpeed;

  std::cout << "Speed: Empty[" << emptySpeed << "] Loaded["
            << loadedSpeed << "] Ratio[" << speedRatio << "]\n";

#ifdef BUILD_TYPE_RELEASE
  EXPECT_GT(speedRatio, 0.5);
#else
  EXPECT_GT(speedRatio, 0.3);
#endif
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
