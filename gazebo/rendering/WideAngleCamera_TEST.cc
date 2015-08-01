

#include <gtest/gtest.h>
#include "gazebo/rendering/WideangleCamera.hh"
#include "gazebo/test/ServerFixture.hh"


using namespace gazebo;
class WideangleCamera_TEST : ServerFixture
{
};


TEST_F(WideangleCamera_TEST, WideangleCameraAPITest)
{
  Load("worlds/empty.world");

  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene("default");

  ASSERT_TRUE(scene != NULL);
}

int int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}