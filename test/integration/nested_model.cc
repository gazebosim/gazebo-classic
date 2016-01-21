/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
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

#include <ignition/math/Pose3.hh>

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
  /// \brief Test loading a world with nested model.
  /// \param[in] _physicsEngine Physics Engine type.
  public: void LoadNestedModel(const std::string &_physicsEngine);

  /// \brief Test spawning a nested model into the world.
  /// \param[in] _physicsEngine Physics Engine type.
  public: void SpawnNestedModel(const std::string &_physicsEngine);
};


////////////////////////////////////////////////////////////////////////
void PhysicsMsgsTest::LoadNestedModel(const std::string &_physicsEngine)
{
  // load a world with a nested model
  Load("worlds/nested_model.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  // take a step to verify that simulation won't crash
  world->Step(1);

  // verify top level model
  physics::ModelPtr model;
  EXPECT_NO_THROW(model = world->GetModel("model_00"));

  if (_physicsEngine == "simbody")
  {
    gzerr << "Nested models are not working in simbody yet, issue #1718"
          << std::endl;
    EXPECT_TRUE(model == NULL);
    return;
  }
  else if (_physicsEngine == "dart")
  {
    gzerr << "Nested models are not working in dart yet, issue #1833"
          << std::endl;

    EXPECT_TRUE(model == NULL);
    return;
  }
  else
  {
    EXPECT_TRUE(model != NULL);
  }

  EXPECT_EQ(model->GetWorldPose().Ign(),
      ignition::math::Pose3d(0, 0, 0.5, 0, 0, 0));

  // verify top level model link
  auto links = model->GetLinks();
  EXPECT_EQ(links.size(), 1u);
  physics::LinkPtr link = links[0];
  EXPECT_TRUE(link != NULL);
  EXPECT_EQ(link->GetName(), "link_00");
  EXPECT_EQ(link->GetScopedName(), "model_00::link_00");
  EXPECT_EQ(link->GetWorldPose().Ign(),
      ignition::math::Pose3d(0, 0, 0.5, 0, 0, 0));

  // verify nested model
  auto models = model->NestedModels();
  EXPECT_EQ(models.size(), 1u);
  physics::ModelPtr nestedModel = models[0];
  EXPECT_TRUE(nestedModel != NULL);
  EXPECT_EQ(nestedModel->GetName(), "model_01");
  EXPECT_EQ(nestedModel->GetScopedName(), "model_00::model_01");
  EXPECT_EQ(nestedModel->GetWorldPose().Ign(),
      ignition::math::Pose3d(1, 0, 0.5, 0, 0, 0));

  // verify nested model link
  auto nestedModelLinks = nestedModel->GetLinks();
  EXPECT_EQ(nestedModelLinks.size(), 1u);
  physics::LinkPtr nestedModelLink = nestedModelLinks[0];
  EXPECT_TRUE(nestedModelLink != NULL);
  EXPECT_EQ(nestedModelLink->GetName(), "link_01");
  EXPECT_EQ(nestedModelLink->GetScopedName(), "model_00::model_01::link_01");
  EXPECT_EQ(nestedModelLink->GetWorldPose().Ign(),
      ignition::math::Pose3d(1.25, 0, 0.5, 0, 0, 0));

  // verify canonical link
  physics::LinkPtr canonicalLink = model->GetLink();
  EXPECT_TRUE(canonicalLink != NULL);
  EXPECT_EQ(canonicalLink->GetName(), "link_00");
  EXPECT_EQ(canonicalLink->GetScopedName(), "model_00::link_00");

  // there should be only one canonical link in the whole model tree
  // check if the nested model's canonical link is the same one as the top
  // level model
  physics::LinkPtr canonicalLink2 = nestedModel->GetLink();
  EXPECT_TRUE(canonicalLink2 != NULL);
  EXPECT_EQ(canonicalLink2->GetName(), "link_00");
  EXPECT_EQ(canonicalLink2->GetScopedName(), "model_00::link_00");

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
  if (_physicsEngine == "simbody")
  {
    gzerr << "Nested models are not working in simbody yet, issue #1718"
          << std::endl;
    return;
  }

  if (_physicsEngine == "dart")
  {
    gzerr << "Nested models are not working in dart yet, issue #1833"
          << std::endl;
    return;
  }

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
    << "    <model name ='model_02'>"
    << "      <pose>0 1.3 0 0 0 0</pose>"
    << "      <link name ='link_01'>"
    << "        <pose>1 0 0 0 0 0</pose>"
    << "        <collision name ='collision_01'>"
    << "          <geometry>"
    << "            <sphere><radius>0.2</radius></sphere>"
    << "          </geometry>"
    << "        </collision>"
    << "        <visual name ='visual_01'>"
    << "          <geometry>"
    << "            <sphere><radius>1 1 1</radius></sphere>"
    << "          </geometry>"
    << "        </visual>"
    << "      </link>"
    << "    </model>"
    << "  </model>"
    << "</model>"
    << "</sdf>";
  SpawnSDF(sdfStream.str());

  // verify top level model
  physics::ModelPtr model;
  model = world->GetModel("model_00");
  EXPECT_TRUE(model != NULL);
  EXPECT_EQ(model->GetWorldPose().Ign(),
      ignition::math::Pose3d(0, 0, 1, 0, 0, 0));

  // verify top level model link
  auto links = model->GetLinks();
  EXPECT_EQ(links.size(), 0u);

  // verify nested model
  auto models = model->NestedModels();
  EXPECT_EQ(models.size(), 1u);
  physics::ModelPtr nestedModel = models[0];
  EXPECT_TRUE(nestedModel != NULL);
  EXPECT_EQ(nestedModel->GetName(), "model_01");
  EXPECT_EQ(nestedModel->GetScopedName(), "model_00::model_01");
  EXPECT_EQ(nestedModel->GetWorldPose().Ign(),
      ignition::math::Pose3d(0, 1, 1, 0, 0, 0));

  // verify nested model links
  auto nestedModelLinks = nestedModel->GetLinks();
  EXPECT_EQ(nestedModelLinks.size(), 2u);
  physics::LinkPtr nestedModelLink = nestedModelLinks[0];
  EXPECT_TRUE(nestedModelLink != NULL);
  EXPECT_EQ(nestedModelLink->GetName(), "link_01");
  EXPECT_EQ(nestedModelLink->GetScopedName(), "model_00::model_01::link_01");
  EXPECT_EQ(nestedModelLink->GetWorldPose().Ign(),
      ignition::math::Pose3d(1, 1, 1, 0, 0, 0));
  physics::LinkPtr nestedModelLink2 = nestedModelLinks[1];
  EXPECT_TRUE(nestedModelLink2 != NULL);
  EXPECT_EQ(nestedModelLink2->GetName(), "link_02");
  EXPECT_EQ(nestedModelLink2->GetScopedName(), "model_00::model_01::link_02");
  EXPECT_EQ(nestedModelLink2->GetWorldPose().Ign(),
      ignition::math::Pose3d(-1, 1, 1, 0, 0, 0));

  // verify nested-nested model
  auto doubleNestedModels = nestedModel->NestedModels();
  EXPECT_EQ(doubleNestedModels.size(), 1u);
  physics::ModelPtr doubleNestedModel = doubleNestedModels[0];
  EXPECT_TRUE(doubleNestedModel != NULL);
  EXPECT_EQ(doubleNestedModel->GetName(), "model_02");
  EXPECT_EQ(doubleNestedModel->GetScopedName(), "model_00::model_01::model_02");
  EXPECT_EQ(doubleNestedModel->GetWorldPose().Ign(),
      ignition::math::Pose3d(0, 2.3, 1, 0, 0, 0));

  // verify nested-nested model links
  auto doubleNestedModelLinks = doubleNestedModel->GetLinks();
  EXPECT_EQ(doubleNestedModelLinks.size(), 1u);
  physics::LinkPtr doubleNestedModelLink = doubleNestedModelLinks[0];
  EXPECT_TRUE(doubleNestedModelLink != NULL);
  EXPECT_EQ(doubleNestedModelLink->GetName(), "link_01");
  EXPECT_EQ(doubleNestedModelLink->GetScopedName(),
      "model_00::model_01::model_02::link_01");
  EXPECT_EQ(doubleNestedModelLink->GetWorldPose().Ign(),
      ignition::math::Pose3d(1, 2.3, 1, 0, 0, 0));

  // verify canonical link
  physics::LinkPtr canonicalLink = model->GetLink();
  EXPECT_TRUE(canonicalLink != NULL);
  EXPECT_EQ(canonicalLink->GetName(), "link_01");
  EXPECT_EQ(canonicalLink->GetScopedName(), "model_00::model_01::link_01");

  // there should be only one canonical link in the whole model tree
  // check if the nested model's canonical link is the same one as the top
  // level model
  physics::LinkPtr canonicalLink2 = nestedModel->GetLink();
  EXPECT_TRUE(canonicalLink2 != NULL);
  EXPECT_EQ(canonicalLink2->GetName(), "link_01");
  EXPECT_EQ(canonicalLink2->GetScopedName(), "model_00::model_01::link_01");

  physics::LinkPtr canonicalLink3 = doubleNestedModel->GetLink();
  EXPECT_TRUE(canonicalLink3 != NULL);
  EXPECT_EQ(canonicalLink3->GetName(), "link_01");
  EXPECT_EQ(canonicalLink3->GetScopedName(), "model_00::model_01::link_01");

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
