/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#include <string.h>

#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/test/ServerFixture.hh"
#include "gazebo/test/helper_physics_generator.hh"

#define PHYSICS_TOL 1e-2
using namespace gazebo;

class PhysicsMsgsTest : public ServerFixture,
                        public testing::WithParamInterface<const char*>
{
  public: void LoadNestedModel(const std::string &_physicsEngine);
  public: void SpawnNestedModel(const std::string &_physicsEngine);
};


////////////////////////////////////////////////////////////////////////
void PhysicsMsgsTest::LoadNestedModel(const std::string &_physicsEngine)
{
  // Nested models are not working in simbody yet
  if (_physicsEngine == "simbody")
    return;

  // load an empty world
  Load("worlds/nested_model.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  // verify top level model
  physics::ModelPtr model;
  model = world->GetModel("model_00");
  EXPECT_TRUE(model != NULL);
  EXPECT_EQ(model->GetWorldPose(), math::Pose(0, 0, 0.5, 0, 0, 0));

  // verify top level model link
  auto links = model->GetLinks();
  EXPECT_EQ(links.size(), 1u);
  physics::LinkPtr link = links[0];
  EXPECT_TRUE(link != NULL);
  EXPECT_EQ(link->GetName(), "link_00");
  EXPECT_EQ(link->GetWorldPose(), math::Pose(0, 0, 0.5, 0, 0, 0));

  // verify nested model
  auto models = model->GetModels();
  EXPECT_EQ(models.size(), 1u);
  physics::ModelPtr nestedModel = models[0];
  EXPECT_TRUE(nestedModel != NULL);
  EXPECT_EQ(nestedModel->GetName(), "model_01");
  EXPECT_EQ(nestedModel->GetWorldPose(), math::Pose(1, 0, 0.5, 0, 0, 0));

  // verify nested model link
  auto nestedModelLinks = nestedModel->GetLinks();
  EXPECT_EQ(nestedModelLinks.size(), 1u);
  physics::LinkPtr nestedModelLink = nestedModelLinks[0];
  EXPECT_TRUE(nestedModelLink != NULL);
  EXPECT_EQ(nestedModelLink->GetName(), "link_01");
  EXPECT_EQ(nestedModelLink->GetWorldPose(), math::Pose(1.25, 0, 0.5, 0, 0, 0));

  // verify canonical link
  physics::LinkPtr canonicalLink = model->GetLink();
  EXPECT_TRUE(canonicalLink != NULL);
  EXPECT_EQ(canonicalLink->GetName(), "link_00");

  // there should be only one canonical link in the whole model tree
  // check if the nested model's canonical link is the same one as the top
  // level model
  physics::LinkPtr canonicalLink2 = nestedModel->GetLink();
  EXPECT_TRUE(canonicalLink2 != NULL);
  EXPECT_EQ(canonicalLink2->GetName(), "link_00");

  // verify model joint
  EXPECT_EQ(model->GetJointCount(), 1u);
  auto joints = model->GetJoints();
  physics::JointPtr joint = joints[0];
  EXPECT_TRUE(joint != NULL);
  EXPECT_EQ(joint->GetName(), "joint_00");
  EXPECT_EQ(joint, model->GetJoint("joint_00"));
  EXPECT_TRUE(joint->GetJointLink(0) != NULL);
  EXPECT_TRUE(joint->GetJointLink(1) != NULL);
  EXPECT_TRUE(joint->GetJointLink(0)->GetName() == "link_00" ||
      joint->GetJointLink(0)->GetName() == "link_01");
  EXPECT_TRUE(joint->GetJointLink(1)->GetName() == "link_00" ||
      joint->GetJointLink(1)->GetName() == "link_01");
  EXPECT_EQ(joint->GetMsgType(), msgs::Joint::REVOLUTE);
  EXPECT_EQ(joint->GetLocalAxis(0), ignition::math::Vector3d::UnitX);

  // verify nested model joint
  EXPECT_EQ(nestedModel->GetJointCount(), 0u);
}

////////////////////////////////////////////////////////////////////////
void PhysicsMsgsTest::SpawnNestedModel(const std::string &_physicsEngine)
{
  // Nested models are not working in simbody yet
  if (_physicsEngine == "simbody")
    return;

  // load an empty world
  Load("worlds/empty.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  std::ostringstream sdfStream;
  sdfStream << "<sdf version='" << SDF_VERSION << "'>"
    << "<model name ='model_00'>"
    << "  <pose>0 0 1 0 0 0</pose>"
    << "  <model name ='model_01'>"
    << "    <pose>0 1 0 0 0 0</pose>"
    << "    <link name ='link_01'>"
    << "      <pose>1 0 0 0 0 0</pose>"
    << "      <collision name ='collision_01'>"
    << "        <geometry>"
    << "          <box><size>1 1 1</size></box>"
    << "        </geometry>"
    << "      </collision>"
    << "      <visual name ='visual_01'>"
    << "        <geometry>"
    << "          <box><size>1 1 1</size></box>"
    << "        </geometry>"
    << "      </visual>"
    << "    </link>"
    << "    <link name ='link_02'>"
    << "      <pose>-1 0 0 0 0 0</pose>"
    << "      <collision name ='collision_02'>"
    << "        <geometry>"
    << "          <cylinder>"
    << "            <radius>0.5</radius>"
    << "            <length>1.0</length>"
    << "          </cylinder>"
    << "        </geometry>"
    << "      </collision>"
    << "      <visual name ='visual_02'>"
    << "        <geometry>"
    << "          <cylinder>"
    << "            <radius>0.5</radius>"
    << "            <length>1.0</length>"
    << "          </cylinder>"
    << "        </geometry>"
    << "      </visual>"
    << "    </link>"
    << "    <joint name ='joint_01' type='prismatic'>"
    << "      <pose>0 1 0 0 0 0</pose>"
    << "      <parent>link_01</parent>"
    << "      <child>link_02</child>"
    << "      <axis>"
    << "        <xyz>0 0 1</xyz>"
    << "        <use_parent_model_frame>true</use_parent_model_frame>"
    << "        <limit>"
    << "          <lower>-0.2</lower>"
    << "          <upper>0.5</upper>"
    << "        </limit>"
    << "      </axis>"
    << "    </joint>"
    << "  </model>"
    << "</model>"
    << "</sdf>";
  SpawnSDF(sdfStream.str());

  // verify top level model
  physics::ModelPtr model;
  model = world->GetModel("model_00");
  EXPECT_TRUE(model != NULL);
  EXPECT_EQ(model->GetWorldPose(), math::Pose(0, 0, 1, 0, 0, 0));

  // verify top level model link
  auto links = model->GetLinks();
  EXPECT_EQ(links.size(), 0u);

  // verify nested model
  auto models = model->GetModels();
  EXPECT_EQ(models.size(), 1u);
  physics::ModelPtr nestedModel = models[0];
  EXPECT_TRUE(nestedModel != NULL);
  EXPECT_EQ(nestedModel->GetName(), "model_01");
  EXPECT_EQ(nestedModel->GetWorldPose(), math::Pose(0, 1, 1, 0, 0, 0));

  // verify nested model links
  auto nestedModelLinks = nestedModel->GetLinks();
  EXPECT_EQ(nestedModelLinks.size(), 2u);
  physics::LinkPtr nestedModelLink = nestedModelLinks[0];
  EXPECT_TRUE(nestedModelLink != NULL);
  EXPECT_EQ(nestedModelLink->GetName(), "link_01");
  EXPECT_EQ(nestedModelLink->GetWorldPose(), math::Pose(1, 1, 1, 0, 0, 0));
  physics::LinkPtr nestedModelLink2 = nestedModelLinks[1];
  EXPECT_TRUE(nestedModelLink2 != NULL);
  EXPECT_EQ(nestedModelLink2->GetName(), "link_02");
  EXPECT_EQ(nestedModelLink2->GetWorldPose(), math::Pose(-1, 1, 1, 0, 0, 0));

  // verify canonical link
  physics::LinkPtr canonicalLink = model->GetLink();
  EXPECT_TRUE(canonicalLink != NULL);
  EXPECT_EQ(canonicalLink->GetName(), "link_01");

  // there should be only one canonical link in the whole model tree
  // check if the nested model's canonical link is the same one as the top
  // level model
  physics::LinkPtr canonicalLink2 = nestedModel->GetLink();
  EXPECT_TRUE(canonicalLink2 != NULL);
  EXPECT_EQ(canonicalLink2->GetName(), "link_01");

  // verify joint
  EXPECT_EQ(model->GetJointCount(), 0u);

  // verify nested model joint
  EXPECT_EQ(nestedModel->GetJointCount(), 1u);
  auto nestedModelJoints = nestedModel->GetJoints();
  physics::JointPtr nestedModelJoint = nestedModelJoints[0];
  EXPECT_TRUE(nestedModelJoint != NULL);
  EXPECT_EQ(nestedModelJoint->GetName(), "joint_01");
  EXPECT_EQ(nestedModelJoint, nestedModel->GetJoint("joint_01"));
  EXPECT_TRUE(nestedModelJoint->GetJointLink(0) != NULL);
  EXPECT_TRUE(nestedModelJoint->GetJointLink(1) != NULL);
  EXPECT_TRUE(nestedModelJoint->GetJointLink(0)->GetName() == "link_01" ||
      nestedModelJoint->GetJointLink(0)->GetName() == "link_02");
  EXPECT_TRUE(nestedModelJoint->GetJointLink(1)->GetName() == "link_01" ||
      nestedModelJoint->GetJointLink(1)->GetName() == "link_02");
  EXPECT_EQ(nestedModelJoint->GetMsgType(), msgs::Joint::PRISMATIC);
  EXPECT_EQ(nestedModelJoint->GetLocalAxis(0), ignition::math::Vector3d::UnitZ);
  EXPECT_EQ(nestedModelJoint->GetLowerLimit(0), -0.2);
  EXPECT_EQ(nestedModelJoint->GetUpperLimit(0), 0.5);
}

/////////////////////////////////////////////////
TEST_P(PhysicsMsgsTest, LoadNestedModel)
{
  LoadNestedModel(GetParam());
}

/////////////////////////////////////////////////
TEST_P(PhysicsMsgsTest, SpawnNestedModel)
{
  SpawnNestedModel(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, PhysicsMsgsTest,
                        PHYSICS_ENGINE_VALUES);

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
