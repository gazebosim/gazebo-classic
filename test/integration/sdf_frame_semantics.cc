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

#include "gazebo/test/ServerFixture.hh"
#include "gazebo/test/helper_physics_generator.hh"

using namespace gazebo;

class SdfFrameSemanticsTest : public ServerFixture,
                              public testing::WithParamInterface<const char *>
{
  public: SdfFrameSemanticsTest()
  {
    this->Load("worlds/empty.world", true);
    this->world = physics::get_world("default");
  }

  public: physics::WorldPtr world;
};

TEST_P(SdfFrameSemanticsTest, LinkRelativeTo)
{
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
  </sdf>
  )sdf";

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
      </joint>
      <joint name="J2" type="revolute">
        <pose relative_to="L2"/>
        <parent>L2</parent>
        <child>L3</child>
      </joint>
    </model>
  </sdf>
  )sdf";

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
  </sdf>
  )sdf";

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
  </sdf>
  )sdf";

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
      </joint>
    </model>
  </sdf>
  )sdf";

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
  </sdf>
  )sdf";

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

INSTANTIATE_TEST_CASE_P(PhysicsEngines, SdfFrameSemanticsTest,
                        PHYSICS_ENGINE_VALUES);

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
