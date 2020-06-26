/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#include <array>
#include <future>
#include <string>

#include <ignition/math/Pose3.hh>

#include "gazebo/common/SystemPaths.hh"
#include "gazebo/test/ServerFixture.hh"
using namespace gazebo;

/////////////////////////////////////////////////
#ifdef _WIN32
static int setenv(const char *envname, const char *envval, int overwrite)
{
  char *original = getenv(envname);
  if (!original || !!overwrite)
  {
    std::string envstring = std::string(envname) + "=" + envval;
    return _putenv(envstring.c_str());
  }
  return 0;
}
#endif

class Issue2728Test : public ServerFixture
{
  public: void CheckPoses()
  {
    physics::WorldPtr world = physics::get_world("default");
    ASSERT_TRUE(world != NULL);

    using ignition::math::Pose3d;

    std::vector<std::pair<physics::LinkPtr, physics::LinkPtr>> testLinks;
    std::array modelNames{"model_urdf", "model_sdf_1_6", "model_sdf_1_7"};

    for (std::string modelName : modelNames)
    {
      // Wait for the models to be spawned
      // Timeout of 30 seconds (3000 * 10 ms)
      int waitCount = 0, maxWaitCount = 3000;
      while (nullptr == world->ModelByName(modelName) &&
             ++waitCount < maxWaitCount)
      {
        common::Time::MSleep(100);
      }

      ASSERT_LT(waitCount, maxWaitCount);

      auto model = world->ModelByName(modelName);
      ASSERT_TRUE(nullptr != model);
      auto links = model->GetLinks();
      ASSERT_EQ(2u, links.size());
      testLinks.push_back(std::make_pair(links[0], links[1]));
    }

    // For each model, the cylinder should be resting on top of the cube
    for (const auto &[boxLink, cylinderLink] : testLinks)
    {
      EXPECT_NEAR(0.1, boxLink->WorldPose().Pos().Z(), 1e-5);
      EXPECT_NEAR(1.1, cylinderLink->WorldPose().Pos().Z(), 1e-5);
    }

    this->SetPause(false);
    world->RunBlocking(1000);

    for (const auto &[boxLink, cylinderLink] : testLinks)
    {
      EXPECT_NEAR(0.0, boxLink->WorldPose().Pos().Z(), 1e-5);
      EXPECT_NEAR(1.0, cylinderLink->WorldPose().Pos().Z(), 1e-5);
    }
  }
};

/////////////////////////////////////////////////
// \brief Test loading world that contains different combinations of SDF verions
// that nest URDF models
TEST_F(Issue2728Test, LoadModelsWithNestedURDF)
{
  // Add the test model database
  gazebo::common::SystemPaths::Instance()->AddModelPathsUpdate(
      PROJECT_SOURCE_PATH "/test/models/test_nested_urdf");

  Load("worlds/sdf_1_6_nested_urdf.world", true);
  this->CheckPoses();
}

TEST_F(Issue2728Test, SpawnModelsWithNestedURDF)
{
  std::string modelPath = PROJECT_SOURCE_PATH "/test/models/test_nested_urdf";
  // We also need to add the model path for the test gzserver
  gazebo::common::SystemPaths::Instance()->AddModelPathsUpdate(modelPath);
  // Add the test model database for gz to be able to find the models
  setenv("GAZEBO_MODEL_PATH", modelPath.c_str(), 0);

  Load("worlds/empty.world", true);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);
  std::stringstream cmd;
  cmd << "gz model -m model_urdf -f " << modelPath <<  "/model_urdf -z 0.1;";
  cmd << "gz model -m model_sdf_1_6 -f " << modelPath << "/model_sdf_1_6 -x 5;";
  cmd << "gz model -m model_sdf_1_7 -f " << modelPath << "/model_sdf_1_7 -x 10";

  custom_exec(cmd.str());

  this->CheckPoses();
}
