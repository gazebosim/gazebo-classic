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

#include "test/util.hh"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/physics/JointState.hh"
#include "gazebo/physics/LinkState.hh"
#include "gazebo/physics/ModelState.hh"

using namespace gazebo;

class ModelStateTest : public gazebo::testing::AutoLogFixture { };

TEST_F(ModelStateTest, Nested)
{
  std::ostringstream sdfStr;
  sdfStr << "<sdf version ='" << SDF_VERSION << "'>"
    << "<world name='default'>"
    << "<state world_name='default'>"
    << "<model name='model_00'>"
    << "  <pose>0 0 0.5 0 0 0</pose>"
    << "  <link name='link_00'>"
    << "    <pose>0 0 0.5 0 0 0</pose>"
    << "    <velocity>0.001 0 0 0 0 0</velocity>"
    << "    <acceleration>0 0.006121 0 0.012288 0 0.001751</acceleration>"
    << "    <wrench>0 0.006121 0 0 0 0</wrench>"
    << "  </link>"
    << "  <model name='model_01'>"
    << "    <pose>1 0 0.5 0 0 0</pose>"
    << "    <link name='link_01'>"
    << "      <pose>1.25 0 0.5 0 0 0</pose>"
    << "      <velocity>0 -0.001 0 0 0 0</velocity>"
    << "      <acceleration>0 0.000674 0 -0.001268 0 0</acceleration>"
    << "      <wrench>0 0.000674 0 0 0 0</wrench>"
    << "    </link>"
    << "    <model name='model_02'>"
    << "      <pose>1 1 0.5 0 0 0</pose>"
    << "      <link name='link_02'>"
    << "        <pose>1.25 1 0.5 0 0 0</pose>"
    << "        <velocity>0 0 0.001 0 0 0</velocity>"
    << "        <acceleration>0 0 0 0 0 0</acceleration>"
    << "        <wrench>0 0 0 0 0 0</wrench>"
    << "      </link>"
    << "    </model>"
    << "  </model>"
    << "</model>"
    << "</state>"
    << "</world>"
    << "</sdf>";

  // load the state sdf
  sdf::SDFPtr worldSDF(new sdf::SDF);
  worldSDF->SetFromString(sdfStr.str());
  EXPECT_TRUE(worldSDF->Root()->HasElement("world"));
  sdf::ElementPtr worldElem = worldSDF->Root()->GetElement("world");
  EXPECT_TRUE(worldElem->HasElement("state"));
  sdf::ElementPtr stateElem = worldElem->GetElement("state");
  EXPECT_TRUE(stateElem->HasElement("model"));

  // create the model state
  physics::ModelState modelState(stateElem->GetElement("model"));

  // fill sdf and load it back to a new model state
  sdf::ElementPtr modelStateElem(new sdf::Element);
  sdf::initFile("model_state.sdf", modelStateElem);
  modelState.FillSDF(modelStateElem);
  physics::ModelState newModelState(modelStateElem);

  // now check both model states against values from the sdf string
  std::vector<physics::ModelState> modelStates;
  modelStates.push_back(modelState);
  modelStates.push_back(newModelState);

  for (const auto &m : modelStates)
  {
    // model state properties
    EXPECT_EQ(m.GetPose(), ignition::math::Pose3d(0, 0, 0.5, 0, 0, 0));
    EXPECT_EQ(m.GetJointStateCount(), 0u);
    EXPECT_EQ(m.GetJointStates().size(), 0u);

    // link state
    EXPECT_EQ(m.GetLinkStateCount(), 1u);
    EXPECT_EQ(m.GetLinkStates().size(), 1u);
    EXPECT_TRUE(m.HasLinkState("link_00"));
    EXPECT_NO_THROW(m.GetLinkState("link_00"));
    physics::LinkState linkState =
        m.GetLinkState("link_00");
    EXPECT_EQ(linkState.GetPose(), ignition::math::Pose3d(0, 0, 0.5, 0, 0, 0));
    EXPECT_EQ(linkState.GetVelocity(),
        ignition::math::Pose3d(0.001, 0, 0, 0, 0, 0));
    EXPECT_EQ(linkState.GetAcceleration(),
        ignition::math::Pose3d(0, 0.006121, 0, 0.012288, 0, 0.001751));
    EXPECT_EQ(linkState.GetWrench(),
        ignition::math::Pose3d(0, 0.006121, 0, 0, 0, 0));

    // nested model state
    EXPECT_TRUE(m.HasNestedModelState("model_01"));
    EXPECT_NO_THROW(m.NestedModelState("model_01"));
    physics::ModelState nestedModelState =
        m.NestedModelState("model_01");

    EXPECT_EQ(nestedModelState.GetPose(),
        ignition::math::Pose3d(1, 0, 0.5, 0, 0, 0));
    EXPECT_EQ(nestedModelState.GetJointStateCount(), 0u);
    EXPECT_EQ(nestedModelState.GetJointStates().size(), 0u);

    // nested model's link state
    EXPECT_EQ(nestedModelState.GetLinkStateCount(), 1u);
    EXPECT_EQ(nestedModelState.GetLinkStates().size(), 1u);
    EXPECT_NO_THROW(nestedModelState.GetLinkState("link_01"));
    physics::LinkState nestedLinkState =
        nestedModelState.GetLinkState("link_01");
    EXPECT_EQ(nestedLinkState.GetPose(),
        ignition::math::Pose3d(1.25, 0, 0.5, 0, 0, 0));
    EXPECT_EQ(nestedLinkState.GetVelocity(),
        ignition::math::Pose3d(0, -0.001, 0, 0, 0, 0));
    EXPECT_EQ(nestedLinkState.GetAcceleration(),
        ignition::math::Pose3d(0, 0.000674, 0, -0.001268, 0, 0));
    EXPECT_EQ(nestedLinkState.GetWrench(),
        ignition::math::Pose3d(0, 0.000674, 0, 0, 0, 0));

    // double nested model state
    EXPECT_TRUE(nestedModelState.HasNestedModelState("model_02"));
    EXPECT_NO_THROW(nestedModelState.NestedModelState("model_02"));
    nestedModelState = nestedModelState.NestedModelState("model_02");

    EXPECT_EQ(nestedModelState.GetPose(),
        ignition::math::Pose3d(1, 1, 0.5, 0, 0, 0));
    EXPECT_EQ(nestedModelState.GetJointStateCount(), 0u);
    EXPECT_EQ(nestedModelState.GetJointStates().size(), 0u);

    // double nested model's link state
    EXPECT_EQ(nestedModelState.GetLinkStateCount(), 1u);
    EXPECT_EQ(nestedModelState.GetLinkStates().size(), 1u);
    EXPECT_TRUE(nestedModelState.HasLinkState("link_02"));
    EXPECT_NO_THROW(nestedModelState.GetLinkState("link_02"));
    nestedLinkState = nestedModelState.GetLinkState("link_02");
    EXPECT_EQ(nestedLinkState.GetPose(),
        ignition::math::Pose3d(1.25, 1, 0.5, 0, 0, 0));
    EXPECT_EQ(nestedLinkState.GetVelocity(),
        ignition::math::Pose3d(0, 0, 0.001, 0, 0, 0));
    EXPECT_EQ(nestedLinkState.GetAcceleration(),
        ignition::math::Pose3d(0, 0, 0, 0, 0, 0));
    EXPECT_EQ(nestedLinkState.GetWrench(),
        ignition::math::Pose3d(0, 0, 0, 0, 0, 0));
  }
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
