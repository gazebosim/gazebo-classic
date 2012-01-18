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
class SpeedTest : public ServerFixture
{
};

TEST_F(SpeedTest, EmptyWorld)
{
  Load("worlds/empty.world");
  double speed = 0;
  while ((speed = GetPercentRealTime()) == 0)
    usleep(100000);

#ifdef BUILD_TYPE_RELEASE
  EXPECT_GT(speed, 3800.0);
#endif
#ifdef BUILD_TYPE_DEBUG
  EXPECT_GT(speed, 800.0);
#endif
  EXPECT_GT(speed, 340.0);
}

TEST_F(SpeedTest, ShapesWorld)
{
  Load("worlds/shapes.world");
  double speed = 0;
  while ((speed = GetPercentRealTime()) == 0)
    usleep(100000);

#ifdef BUILD_TYPE_RELEASE
  EXPECT_GT(speed, 110.0);
#endif
#ifdef BUILD_TYPE_DEBUG
  EXPECT_GT(speed, 25.0);
#endif
  EXPECT_GT(speed, 18.0);
}

TEST_F(SpeedTest, PR2World)
{
  Load("worlds/pr2.world");
  double speed = 0;
  while ((speed = GetPercentRealTime()) == 0)
    usleep(100000);

#ifdef BUILD_TYPE_RELEASE
  EXPECT_GT(speed, 4.0);
#endif
#ifdef BUILD_TYPE_DEBUG
  EXPECT_GT(speed, 1.0);
#endif
  EXPECT_GT(speed, 0.4);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
