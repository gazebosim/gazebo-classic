/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include <chrono>

#include "gazebo/test/ServerFixture.hh"
#include "gazebo/test/helper_physics_generator.hh"

using namespace gazebo;

class SdfFrameSemanticsTest : public ServerFixture,
                              public testing::WithParamInterface<const char *>
{
  public: void LoadWorld(const std::string &_worldFile = "worlds/empty.world")
  {
    this->Load(_worldFile, true, GetParam());
    this->world = physics::get_world("default");
  }

  public: physics::WorldPtr world;
};

TEST_P(SdfFrameSemanticsTest, LinkRelativeTo)
{
  this->LoadWorld();
  const std::string modelSdf = R"sdf(
  <sdf version="1.7">
    <model name="M">
      <pose>0.0 0 2 0 0 0</pose>
      <link name="L1">
        <pose>0.5 0 0 0 0 3.14159265358979</pose>
      </link>
      <link name="L2">
        <pose relative_to="L1">0.5 0 1 0 0 0</pose>
      </link>
    </model>
  </sdf>)sdf";

  this->SpawnSDF(modelSdf);

  auto model = this->GetModel("M");
  ASSERT_NE(nullptr, model);

  auto link2 = model->GetLink("L2");
  ASSERT_NE(nullptr, link2);
  // Expect the pose of L2 relative to model to be 0 0 1 0 0 pi
  ignition::math::Pose3d expRelPose(0, 0, 1, 0, 0, IGN_PI);
  ignition::math::Pose3d expWorldPose(0, 0, 3, 0, 0, IGN_PI);
  EXPECT_EQ(expRelPose, link2->RelativePose());
  EXPECT_EQ(expWorldPose, link2->WorldPose());
  // Step once and check, the pose should still be close to its initial pose
  this->world->Step(1);

  EXPECT_EQ(expRelPose, link2->RelativePose());
  EXPECT_EQ(expWorldPose, link2->WorldPose());
}

TEST_P(SdfFrameSemanticsTest, LinkRelativeToJoint)
{
  this->LoadWorld();
  const std::string modelSdf = R"sdf(
  <sdf version="1.7">
    <model name="M">
      <pose>0.0 0 2 0 0 0</pose>
      <link name="L1">
        <pose>0.5 0 0 0 0 3.14159265358979</pose>
      </link>
      <link name="L2">
        <pose relative_to="J1"/>
      </link>
      <joint name="J1" type="revolute">
        <pose relative_to="L1">0.5 0 1 0 0 0</pose>
        <parent>L1</parent>
        <child>L2</child>
        <axis>
          <xyz>0 0 1</xyz>
        </axis>
      </joint>
    </model>
  </sdf>)sdf";

  this->SpawnSDF(modelSdf);

  auto model = this->GetModel("M");
  ASSERT_NE(nullptr, model);

  auto link2 = model->GetLink("L2");
  ASSERT_NE(nullptr, link2);
  // Expect the pose of L2 relative to model to be 0 0 1 0 0 pi
  ignition::math::Pose3d expRelPose(0, 0, 1, 0, 0, IGN_PI);
  ignition::math::Pose3d expWorldPose(0, 0, 3, 0, 0, IGN_PI);
  EXPECT_EQ(expRelPose, link2->RelativePose());
  EXPECT_EQ(expWorldPose, link2->WorldPose());
  // Step once and check, the pose should still be close to its initial pose
  this->world->Step(1);

  EXPECT_EQ(expRelPose, link2->RelativePose());
  EXPECT_EQ(expWorldPose, link2->WorldPose());
}

TEST_P(SdfFrameSemanticsTest, JointRelativeTo)
{
  this->LoadWorld();
  const std::string modelSdf = R"sdf(
  <sdf version="1.7">
    <model name="M">
      <pose>1.0 0 0 0 0 0</pose>
      <link name="L1">
        <pose>0.0 0 1 0 0 0</pose>
      </link>
      <link name="L2">
        <pose>0.0 0 2 0 0 0</pose>
      </link>
      <link name="L3">
        <pose>0.0 0 3 0 0 0</pose>
      </link>
      <joint name="J1" type="revolute">
        <parent>L1</parent>
        <child>L2</child>
        <axis>
          <xyz>0 0 1</xyz>
        </axis>
      </joint>
      <joint name="J2" type="revolute">
        <pose relative_to="L2"/>
        <parent>L2</parent>
        <child>L3</child>
        <axis>
          <xyz>0 0 1</xyz>
        </axis>
      </joint>
    </model>
  </sdf>)sdf";

  this->SpawnSDF(modelSdf);

  auto model = this->GetModel("M");
  ASSERT_NE(nullptr, model);

  auto link2 = model->GetLink("L2");
  ASSERT_NE(nullptr, link2);
  auto joint1 = model->GetJoint("J1");
  ASSERT_NE(nullptr, joint1);
  auto joint2 = model->GetJoint("J2");
  ASSERT_NE(nullptr, joint2);
  // Expect the pose of J1 relative to model to be the same as L2 (default
  // behavior)
  ignition::math::Pose3d expWorldPose(1, 0, 2, 0, 0, 0);
  EXPECT_EQ(expWorldPose, joint1->WorldPose());

  // Expect the pose of J2 relative to model to be the same as L2 (non default
  // behavior due to "relative_to='L2'")
  EXPECT_EQ(expWorldPose, joint2->WorldPose());
  // Step once and check, the poses should still be close to their initial pose
  this->world->Step(1);

  EXPECT_EQ(expWorldPose, joint1->WorldPose());
  EXPECT_EQ(expWorldPose, joint2->WorldPose());
}

TEST_P(SdfFrameSemanticsTest, VisualCollisionRelativeTo)
{
  this->LoadWorld();
  const std::string modelSdf = R"sdf(
  <sdf version="1.7">
    <model name="M">
      <pose>1.0 0 0 0 0 0</pose>
      <link name="L1">
        <pose>0.0 0 1 0 0 0</pose>
      </link>
      <link name="L2">
        <pose>0.0 0 2 0 0 0</pose>
        <visual name="v1">
          <pose relative_to="L1"/>
          <geometry><sphere><radius>1</radius></sphere></geometry>
        </visual>
        <collision name="c1">
          <pose relative_to="L1"/>
          <geometry><sphere><radius>1</radius></sphere></geometry>
        </collision>
      </link>
    </model>
  </sdf>)sdf";

  this->SpawnSDF(modelSdf);

  auto model = this->GetModel("M");
  ASSERT_NE(nullptr, model);

  auto link2 = model->GetLink("L2");
  ASSERT_NE(nullptr, link2);
  uint32_t visualId;
  ASSERT_TRUE(link2->VisualId("v1", visualId));
  auto collision = link2->GetCollision("c1");
  ASSERT_NE(nullptr, collision);

  // Expect the pose of v1 and relative to L2 (their parent link) to be the same
  // as the pose of L1 relative to L2
  ignition::math::Pose3d expPose(0, 0, -1, 0, 0, 0);
  {
    ignition::math::Pose3d visPose;
    EXPECT_TRUE(link2->VisualPose(visualId, visPose));
    EXPECT_EQ(expPose, visPose);
  }
  EXPECT_EQ(expPose, collision->RelativePose());
  // Step once and check, the pose should still be close to its initial pose
  this->world->Step(1);

  {
    ignition::math::Pose3d visPose;
    EXPECT_TRUE(link2->VisualPose(visualId, visPose));
    EXPECT_EQ(expPose, visPose);
  }
  EXPECT_EQ(expPose, collision->RelativePose());
}

TEST_P(SdfFrameSemanticsTest, ExplicitFramesWithLinks)
{
  this->LoadWorld();
  const std::string modelSdf = R"sdf(
  <sdf version="1.7">
    <model name="M">
      <pose>1.0 0 0 0 0 0</pose>
      <frame name="F1">
        <pose>0.0 0 1 0 0 0</pose>
      </frame>
      <link name="L1">
        <pose relative_to="F1"/>
      </link>

      <frame name="F2" attached_to="L1">
        <pose relative_to="__model__">0.0 0 0 0 0 0</pose>
      </frame>
      <link name="L2">
        <pose relative_to="F2"/>
      </link>
    </model>
  </sdf>)sdf";

  this->SpawnSDF(modelSdf);

  auto model = this->GetModel("M");
  ASSERT_NE(nullptr, model);

  auto link1 = model->GetLink("L1");
  ASSERT_NE(nullptr, link1);

  auto link2 = model->GetLink("L2");
  ASSERT_NE(nullptr, link2);

  // Expect the pose of L1 and relative to M to be the same
  // as the pose of F1 relative to M
  ignition::math::Pose3d link1ExpRelativePose(0, 0, 1, 0, 0, 0);
  EXPECT_EQ(link1ExpRelativePose, link1->RelativePose());

  // Expect the pose of L2 and relative to M to be the same
  // as the pose of F2, which is at the origin of M
  ignition::math::Pose3d link2ExpRelativePose = ignition::math::Pose3d::Zero;
  EXPECT_EQ(link2ExpRelativePose, link2->RelativePose());

  // Step once and check, the pose should still be close to its initial pose
  this->world->Step(1);

  EXPECT_EQ(link1ExpRelativePose, link1->RelativePose());
  EXPECT_EQ(link2ExpRelativePose, link2->RelativePose());
}

TEST_P(SdfFrameSemanticsTest, ExplicitFramesWithJoints)
{
  this->LoadWorld();
  const std::string modelSdf = R"sdf(
  <sdf version="1.7">
    <model name="M">
      <pose>1.0 0 0 0 0 0</pose>
      <frame name="F1">
        <pose>0.0 0 2 0 0 0</pose>
      </frame>
      <link name="L1">
        <pose relative_to="F1"/>
      </link>
      <link name="L2">
        <pose>0 1 0 0 0 0</pose>
      </link>
      <joint name="J1" type="revolute">
        <pose relative_to="F1"/>
        <parent>L1</parent>
        <child>L2</child>
        <axis>
          <xyz>0 0 1</xyz>
        </axis>
      </joint>
    </model>
  </sdf>)sdf";

  this->SpawnSDF(modelSdf);

  auto model = this->GetModel("M");
  ASSERT_NE(nullptr, model);

  auto joint1 = model->GetJoint("J1");
  ASSERT_NE(nullptr, joint1);

  // Expect the pose of J1 relative to model to be the same as F1 in world
  ignition::math::Pose3d expWorldPose(1, 0, 2, 0, 0, 0);
  EXPECT_EQ(expWorldPose, joint1->WorldPose());
  // Step once and check, the poses should still be close to their initial pose
  this->world->Step(1);

  EXPECT_EQ(expWorldPose, joint1->WorldPose());
}

TEST_P(SdfFrameSemanticsTest, ExplicitFramesWithVisualAndCollision)
{
  this->LoadWorld();
  const std::string modelSdf = R"sdf(
  <sdf version="1.7">
    <model name="M">
      <pose>1.0 0 0 0 0 0</pose>
      <frame name="F1">
        <pose relative_to="L1">0 0 1 0 0 0</pose>
      </frame>
      <link name="L1">
        <pose>0 0 2 0 0 0</pose>
        <visual name="v1">
          <pose relative_to="F1"/>
          <geometry><sphere><radius>1</radius></sphere></geometry>
        </visual>
        <collision name="c1">
          <pose relative_to="F1"/>
          <geometry><sphere><radius>1</radius></sphere></geometry>
        </collision>
      </link>
    </model>
  </sdf>)sdf";

  this->SpawnSDF(modelSdf);

  auto model = this->GetModel("M");
  ASSERT_NE(nullptr, model);

  auto link1 = model->GetLink("L1");
  ASSERT_NE(nullptr, link1);
  uint32_t visualId;
  ASSERT_TRUE(link1->VisualId("v1", visualId));
  auto collision = link1->GetCollision("c1");
  ASSERT_NE(nullptr, collision);

  // Expect the pose of v1 and relative to L1 (their parent link) to be the same
  // as the pose of F1 relative to L1
  ignition::math::Pose3d expPose(0, 0, 1, 0, 0, 0);
  {
    ignition::math::Pose3d visPose;
    EXPECT_TRUE(link1->VisualPose(visualId, visPose));
    EXPECT_EQ(expPose, visPose);
  }
  EXPECT_EQ(expPose, collision->RelativePose());
  // Step once and check, the pose should still be close to its initial pose
  this->world->Step(1);

  {
    ignition::math::Pose3d visPose;
    EXPECT_TRUE(link1->VisualPose(visualId, visPose));
    EXPECT_EQ(expPose, visPose);
  }
  EXPECT_EQ(expPose, collision->RelativePose());
}

//////////////////////////////////////////////////
/// Test that it is possible to spawn multiple models each with different sdf
/// versions.
TEST_P(SdfFrameSemanticsTest, MultipleSDFVersionsCoexist)
{
  this->LoadWorld();
  const std::string modelSDF17 = R"sdf(
  <sdf version="1.7">
    <model name="M1">
      <link name="L"/>
    </model>
  </sdf>)sdf";

  // Valid in 1.6, but not in 1.7 because a canonical link is missing
  const std::string modelSDF16 = R"sdf(
  <sdf version="1.6">
    <model name="M2">
    </model>
  </sdf>)sdf";

  auto tInit = std::chrono::high_resolution_clock::now();
  this->SpawnSDF(modelSDF17);
  auto tFin = std::chrono::high_resolution_clock::now();
  int dur = std::chrono::duration_cast<std::chrono::milliseconds>(tFin - tInit)
                .count();

  // Use a factory message instead of SpawnSDF to avoid blocking for a long time
  // since we expect (in the failing case) for this to not load successfully.
  msgs::Factory msg;
  msg.set_sdf(modelSDF16);
  this->factoryPub->Publish(msg);
  // maxWaitCount is the maximum of 5s and 4 times the time it took to spawn
  // modelSDF17
  int maxWaitCount = std::max(dur * 4, 5000) / 1000;
  int waitCount = 0;
  while (!this->HasEntity("M2") && ++waitCount < maxWaitCount)
  {
    using namespace std::chrono_literals;
    std::this_thread::sleep_for(1000ms);
  }

  EXPECT_NE(nullptr, this->GetModel("M1"));
  EXPECT_NE(nullptr, this->GetModel("M2"));
}

TEST_P(SdfFrameSemanticsTest, ImplicitModelFrames)
{
  this->LoadWorld("worlds/frame_semantics_implicit_model_frames.world");

  auto model1 = this->GetModel("M1");
  ASSERT_NE(nullptr, model1);

  auto model2 = this->GetModel("M2");
  ASSERT_NE(nullptr, model2);

  auto link1 = model1->GetLink("L1");
  ASSERT_NE(nullptr, link1);

  auto link2 = model2->GetLink("L2");
  ASSERT_NE(nullptr, link2);

  // Expect the world pose of L1 and L2 to be identical and at the origin
  ignition::math::Pose3d expM1WorldPose(0, 0, 0, 0, 0, IGN_PI);
  EXPECT_EQ(expM1WorldPose, link1->WorldPose());
  EXPECT_EQ(expM1WorldPose, link2->WorldPose());

  // Expect the relative pose (and the world pose) of M2 to be -0.5 meters in
  // the -y direction
  ignition::math::Pose3d expM2Pose(0.5, -0.5, 0, 0, 0, IGN_PI);
  EXPECT_EQ(expM2Pose, model2->RelativePose());
  EXPECT_EQ(expM2Pose, model2->WorldPose());
  // Step once and check, the pose should still be close to its initial pose
  this->world->Step(1);

  EXPECT_EQ(expM1WorldPose, link1->WorldPose());
  EXPECT_EQ(expM1WorldPose, link2->WorldPose());
  EXPECT_EQ(expM2Pose, model2->RelativePose());
  EXPECT_EQ(expM2Pose, model2->WorldPose());
}

TEST_P(SdfFrameSemanticsTest, JointAxisXyzExpressedIn)
{
  this->LoadWorld();
  const std::string modelSdf = R"sdf(
  <sdf version="1.7">
    <model name="M">
      <pose>1.0 0 0 0 0 0</pose>
      <link name="L1">
        <pose>0 0 1 0 -1.5707963267948966 0</pose>
      </link>
      <link name="L2">
        <pose>0.0 0 2 0 0 0</pose>
      </link>
      <link name="L3">
        <pose>0.0 0 3 3.14159265358979 0 0</pose>
      </link>
      <joint name="J1" type="revolute">
        <parent>L1</parent>
        <child>L2</child>
        <axis>
          <xyz expressed_in="L1">1 0 0</xyz>
        </axis>
      </joint>
      <joint name="J2" type="revolute">
        <parent>L2</parent>
        <child>L3</child>
        <axis>
          <xyz expressed_in="__model__">0 0 1</xyz>
        </axis>
      </joint>
    </model>
  </sdf>)sdf";

  this->SpawnSDF(modelSdf);

  auto model = this->GetModel("M");
  ASSERT_NE(nullptr, model);

  auto link2 = model->GetLink("L2");
  ASSERT_NE(nullptr, link2);
  auto joint1 = model->GetJoint("J1");
  ASSERT_NE(nullptr, joint1);
  auto joint2 = model->GetJoint("J2");
  ASSERT_NE(nullptr, joint2);
  // Expect the xyz of both joints to point in the world +z direction
  ignition::math::Vector3d expGlobalAxis(0, 0, 1);
  EXPECT_EQ(expGlobalAxis, joint1->GlobalAxis(0));
  EXPECT_EQ(expGlobalAxis, joint2->GlobalAxis(0));

  // Step once and check, the vector should still be close to their initial
  // values
  this->world->Step(1);

  EXPECT_EQ(expGlobalAxis, joint1->GlobalAxis(0));
  EXPECT_EQ(expGlobalAxis, joint2->GlobalAxis(0));
}

TEST_P(SdfFrameSemanticsTest, ExplicitWorldFrames)
{
  this->LoadWorld("worlds/frame_semantics_explicit_world_frames.world");

  auto model1 = this->GetModel("M1");
  ASSERT_NE(nullptr, model1);

  auto model2 = this->GetModel("M2");
  ASSERT_NE(nullptr, model2);

  auto link1 = model1->GetLink("L1");
  ASSERT_NE(nullptr, link1);

  auto link2 = model2->GetLink("L2");
  ASSERT_NE(nullptr, link2);

  // Expect the world pose of L1 and L2 to be identical but translated 10m in z.
  ignition::math::Pose3d expM1WorldPose(0, 0, 10, 0, 0, IGN_PI);
  EXPECT_EQ(expM1WorldPose, link1->WorldPose());
  EXPECT_EQ(expM1WorldPose, link2->WorldPose());

  // Expect the relative pose (and the world pose) of M2 to be -0.5 meters in
  // the -y direction
  ignition::math::Pose3d expM2Pose(0.5, -0.5, 10, 0, 0, IGN_PI);
  EXPECT_EQ(expM2Pose, model2->RelativePose());
  EXPECT_EQ(expM2Pose, model2->WorldPose());
  // Step once and check, the pose should still be close to its initial pose
  this->world->Step(1);

  EXPECT_EQ(expM1WorldPose, link1->WorldPose());
  EXPECT_EQ(expM1WorldPose, link2->WorldPose());
  EXPECT_EQ(expM2Pose, model2->RelativePose());
  EXPECT_EQ(expM2Pose, model2->WorldPose());
}

TEST_P(SdfFrameSemanticsTest, SensorPose)
{
  this->LoadWorld("worlds/frame_semantics_sensor_pose.world");

  sensors::ImuSensorPtr imu =
    std::static_pointer_cast<sensors::ImuSensor>(
    sensors::SensorManager::Instance()->GetSensor("imu_sensor"));
  ASSERT_TRUE(imu != NULL);
  imu->Init();

  sensors::ForceTorqueSensorPtr ftSensor =
    std::static_pointer_cast<sensors::ForceTorqueSensor>(
    sensors::SensorManager::Instance()->GetSensor("force_torque_sensor"));
  ASSERT_TRUE(ftSensor != NULL);
  ftSensor->Init();

  ignition::math::Pose3d expImuSensorPose(0, 0.2, 0, IGN_PI/2, 0, 0);
  EXPECT_EQ(expImuSensorPose, imu->Pose());

  ignition::math::Pose3d expFtSensorPose(1, 0, 0, 0, IGN_PI/2, 0);
  EXPECT_EQ(expFtSensorPose, ftSensor->Pose());

  this->world->Step(1);

  EXPECT_EQ(expImuSensorPose, imu->Pose());
  EXPECT_EQ(expFtSensorPose, ftSensor->Pose());
}


TEST_P(SdfFrameSemanticsTest, IncludedModel)
{
  this->LoadWorld("worlds/frame_semantics_included_model.world");

  auto model1 = this->GetModel("M1");
  ASSERT_NE(nullptr, model1);

  auto model2 = this->GetModel("M2");
  ASSERT_NE(nullptr, model2);

  // Expect the world pose of the M1 to be the same as frame F_I
  ignition::math::Pose3d expM1WorldPose(5, 0, 5, 0, 0, 0);
  EXPECT_EQ(expM1WorldPose, model1->WorldPose());

  // Expect the world pose of the M2 to be offset from M1 by 1m in the y
  // direction
  ignition::math::Pose3d expM2WorldPose(5, 1, 5, 0, 0, 0);
  EXPECT_EQ(expM2WorldPose, model2->WorldPose());

  // Step once and check, the poses should still be close to its initial pose
  this->world->Step(1);

  EXPECT_EQ(expM1WorldPose, model1->WorldPose());
  EXPECT_EQ(expM2WorldPose, model2->WorldPose());
}

/////////////////////////////////////////////////
/// Helper function to check poses in Nested model with frame semantics tests.
void checkPoses(physics::WorldPtr _world)
{
  using ignition::math::Pose3d;

  physics::ModelPtr parentModel = _world->ModelByName("parent_model");
  ASSERT_TRUE(nullptr != parentModel);
  EXPECT_EQ(Pose3d::Zero, parentModel->WorldPose());

  auto checkPose = [&](const std::string &_name, const Pose3d &_pose)
  {
    auto entity = boost::dynamic_pointer_cast<physics::Entity>(
        parentModel->GetByName(_name));
    ASSERT_TRUE(nullptr != entity) << "Entity [" << _name << "] not found";
    EXPECT_EQ(_pose, entity->WorldPose());
  };

  checkPose("parent_model::M1", {1, 0, 0, 0, IGN_PI_2, 0});
  checkPose("parent_model::M1::L", {2, 0, 0, 0, IGN_PI_2, 0});
  checkPose("parent_model::M2", {2, 0, 0, 0, 0, 0});
  checkPose("parent_model::M2::L", {2, 0, 1, 0, 0, 0});
  checkPose("parent_model::M3", {1, 0, -3, 0, IGN_PI_2, 0});
  checkPose("parent_model::M3::L", {2, 0, -3, 0, IGN_PI_2, 0});

  // Joints are not "Entity"s
  auto j1 = parentModel->GetJoint("J1");
  ASSERT_FALSE(nullptr == j1);
  // World pose of F1 = 1, 1, -3, 0, IGN_PI_2, 0
  EXPECT_EQ(Pose3d(1, 1, -4, 0, IGN_PI_2, 0), j1->WorldPose());
}

/////////////////////////////////////////////////
TEST_P(SdfFrameSemanticsTest, LoadNestedModelWithFrameSemantics)
{
  const std::string physicsEngine = GetParam();
  if (physicsEngine == "simbody")
  {
    gzerr << "Nested models are not working in simbody yet, issue #1718"
          << std::endl;
    return;
  }
  else if (physicsEngine == "dart")
  {
    gzerr << "Nested models are not working in dart yet, issue #1833"
          << std::endl;
    return;
  }

  // load a world with a nested model
  this->LoadWorld("test/worlds/nested_model_with_frame_semantics.sdf");
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(nullptr != world);

  // take a step to verify that simulation won't crash
  world->Step(1);

  checkPoses(world);
}

////////////////////////////////////////////////////////////////////////
TEST_P(SdfFrameSemanticsTest, SpawnNestedModelWithFrameSemantics)
{
  const std::string physicsEngine = GetParam();
  if (physicsEngine == "simbody")
  {
    gzerr << "Nested models are not working in simbody yet, issue #1718"
          << std::endl;
    return;
  }

  if (physicsEngine == "dart")
  {
    gzerr << "Nested models are not working in dart yet, issue #1833"
          << std::endl;
    return;
  }

  this->LoadWorld();

  const std::string inputSdf = R"(
  <sdf version="1.7">
    <model name="parent_model">
      <link name="L1"/>
      <link name="L2">
        <pose>0 1 0 0 0 0</pose>
      </link>
      <model name="M1">
        <pose>1 0 0 0 1.5707963267948966 0</pose>
        <link name="L">
          <pose>0 0 1 0 0 0</pose>
        </link>
      </model>
      <model name="M2">
        <pose relative_to="">2 0 0 0 0 0</pose>
        <link name="L">
          <pose>0 0 1 0 0 0</pose>
        </link>
      </model>
      <model name="M3">
        <pose relative_to="M1">3 0 0 0 0 0</pose>
        <link name="L">
          <pose>0 0 1 0 0 0</pose>
        </link>
      </model>

      <frame name="F1">
        <pose relative_to="M3">0 1 0 0 0 0</pose>
      </frame>

      <joint name="J1" type="revolute">
        <pose relative_to="F1">1 0 0 0 0 0</pose>
        <parent>L1</parent>
        <child>L2</child>
        <axis>
          <xyz>1 0 0</xyz>
        </axis>
      </joint>
    </model>
  </sdf>)";

  this->SpawnSDF(inputSdf);

  physics::WorldPtr world = physics::get_world();
  ASSERT_TRUE(nullptr != world);
  world->Step(1);
  checkPoses(world);
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, SdfFrameSemanticsTest,
                        PHYSICS_ENGINE_VALUES,); // NOLINT

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
