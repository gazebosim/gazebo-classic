#include "ServerFixture.hh"

using namespace gazebo;
class SpeedTest : public ServerFixture
{};
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
#ifdef BUILD_TYPE_PROFILE
  EXPECT_GT(speed, 340.0);
#endif

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
#ifdef BUILD_TYPE_PROFILE
  EXPECT_GT(speed, 18.0);
#endif

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
#ifdef BUILD_TYPE_PROFILE
  EXPECT_GT(speed, 0.4);
#endif
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

