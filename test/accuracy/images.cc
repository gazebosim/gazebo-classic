/*
 * Copyright (C) 2012-2013 Open Source Robotics Foundation
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

#include <gtest/gtest.h>

#include "gazebo/util/ZadeatGTParser.hh"
#include "gazebo/util/ZadeatImgParser.hh"
#include "ServerFixture.hh"

using namespace gazebo;

class ImageAccuracyTest : public ServerFixture
{
};

/////////////////////////////////////////////////
TEST_F(ImageAccuracyTest, ZadeatDataset)
{
  /*gazebo::util::ZadeatImgParser *imgParser = new gazebo::util::ZadeatImgParser
    ("/opt/naoDataset/run02/camera_2.strm", "/opt/naoDataset/run02/images/");

  imgParser->Parse();*/

  Load("worlds/empty.world");

  double timestamp;
  double headRoll, headPitch, headYaw;
  double headX, headY, headZ;
  double torsoRoll, torsoPitch, torsoYaw;
  double torsoX, torsoY, torsoZ;

  gazebo::util::ZadeatGTParser *gtParser = new gazebo::util::ZadeatGTParser(
    "/data/naoDataset/run0/ground_truth_0.csv");

  /*while (gtParser->GetNextGT(timestamp, headRoll, headPitch, headYaw,
                              headX, headY, headZ,
                              torsoRoll, torsoPitch, torsoYaw,
                              torsoX, torsoY, torsoZ))
  {*/
  
  gtParser->GetNextGT(timestamp, headRoll, headPitch, headYaw,
                              headX, headY, headZ,
                              torsoRoll, torsoPitch, torsoYaw,
                              torsoX, torsoY, torsoZ);

  // Move the Gazebo camera to that position
  //math::Pose cameraPose(headX, headY, headZ, headRoll, headPitch, headYaw);

  math::Pose cameraPose(0, 0, 0, 0, 0, 0);

  // Spawn a camera that will do the following
  SpawnCamera("test_camera_model", "test_camera",
      cameraPose.pos, cameraPose.rot.GetAsEuler());

  WaitUntilEntitySpawn("test_camera_model", 100, 50);
  WaitUntilSensorSpawn("test_camera", 100, 100); 

  // Take a snapshot
  rendering::ScenePtr scene = rendering::get_scene();
  ASSERT_TRUE(scene);

  rendering::CameraPtr camera = scene->GetCamera("test_camera");
  ASSERT_TRUE(camera);
 
  camera->Render(true);
  camera->PostRender();
  
  // Store the image
  camera->SaveFrame("/tmp/testImage.png");
  //}

}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
