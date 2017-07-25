/*
 * Copyright (C) 2017 Google, Inc.
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

using namespace gazebo;

class JointStateTest : public gazebo::testing::AutoLogFixture { };

TEST_F(JointStateTest, DefaultConstructedJointStateHasNoAngles)
{
  physics::JointState jointState;

  EXPECT_EQ(0u, jointState.GetAngleCount());
  EXPECT_EQ(0u, jointState.Positions().size());
  EXPECT_TRUE(ignition::math::isnan(jointState.Position(0)));
}

TEST_F(JointStateTest, LoadFromSdfSingleAxis)
{
  std::ostringstream sdfStr;
  sdfStr << "<sdf version ='" << SDF_VERSION << "'>"
    << "<world name='default'>"
    << "<state world_name='default'>"
    << "<model name='model_00'>"
    << "  <joint name='joint_00'>"
    << "    <angle axis='0'>1.57</angle>"
    << "  </joint>"
    << "</model>"
    << "</state>"
    << "</world>"
    << "</sdf>";
  sdf::SDFPtr worldSDF(new sdf::SDF);
  worldSDF->SetFromString(sdfStr.str());
  EXPECT_TRUE(worldSDF->Root()->HasElement("world"));
  sdf::ElementPtr worldElem = worldSDF->Root()->GetElement("world");
  EXPECT_TRUE(worldElem->HasElement("state"));
  sdf::ElementPtr stateElem = worldElem->GetElement("state");
  EXPECT_TRUE(stateElem->HasElement("model"));
  sdf::ElementPtr modelElem = stateElem->GetElement("model");
  EXPECT_TRUE(modelElem != nullptr);
  sdf::ElementPtr jointElem = modelElem->GetElement("joint");
  EXPECT_TRUE(jointElem != nullptr);

  physics::JointState jointState;
  jointState.Load(jointElem);

  EXPECT_EQ(1u, jointState.GetAngleCount());
  EXPECT_EQ(1u, jointState.Positions().size());
  EXPECT_DOUBLE_EQ(1.57, jointState.Position(0));
  EXPECT_TRUE(ignition::math::isnan(jointState.Position(1)));
}

TEST_F(JointStateTest, LoadFromSdfMultiAxis)
{
  std::ostringstream sdfStr;
  sdfStr << "<sdf version ='" << SDF_VERSION << "'>"
    << "<world name='default'>"
    << "<state world_name='default'>"
    << "<model name='model_00'>"
    << "  <joint name='joint_00'>"
    << "    <angle axis='0'>1.57</angle>"
    << "    <angle axis='1'>3.14</angle>"
    << "  </joint>"
    << "</model>"
    << "</state>"
    << "</world>"
    << "</sdf>";
  sdf::SDFPtr worldSDF(new sdf::SDF);
  worldSDF->SetFromString(sdfStr.str());
  EXPECT_TRUE(worldSDF->Root()->HasElement("world"));
  sdf::ElementPtr worldElem = worldSDF->Root()->GetElement("world");
  EXPECT_TRUE(worldElem->HasElement("state"));
  sdf::ElementPtr stateElem = worldElem->GetElement("state");
  EXPECT_TRUE(stateElem->HasElement("model"));
  sdf::ElementPtr modelElem = stateElem->GetElement("model");
  EXPECT_TRUE(modelElem != nullptr);
  sdf::ElementPtr jointElem = modelElem->GetElement("joint");
  EXPECT_TRUE(jointElem != nullptr);

  physics::JointState jointState;
  jointState.Load(jointElem);

  EXPECT_EQ(2u, jointState.GetAngleCount());
  EXPECT_EQ(2u, jointState.Positions().size());
  EXPECT_DOUBLE_EQ(1.57, jointState.Position(0));
  EXPECT_DOUBLE_EQ(3.14, jointState.Position(1));
  EXPECT_TRUE(ignition::math::isnan(jointState.Position(2)));
}

TEST_F(JointStateTest, StreamDefaultConstructed)
{
  physics::JointState defaultJointState;

  std::ostringstream sdfStr;
  sdfStr << "<sdf version ='" << SDF_VERSION << "'>"
  << "<world name='default'>"
  << "<state world_name='default'>"
  << "<model name='model_00'>"
  << defaultJointState
  << "</model>"
  << "</state>"
  << "</world>"
  << "</sdf>";

  sdf::SDFPtr worldSDF(new sdf::SDF);
  worldSDF->SetFromString(sdfStr.str());
  EXPECT_TRUE(worldSDF->Root()->HasElement("world"));
  sdf::ElementPtr worldElem = worldSDF->Root()->GetElement("world");
  EXPECT_TRUE(worldElem->HasElement("state"));
  sdf::ElementPtr stateElem = worldElem->GetElement("state");
  EXPECT_TRUE(stateElem->HasElement("model"));
  sdf::ElementPtr modelElem = stateElem->GetElement("model");
  EXPECT_TRUE(modelElem != nullptr);
  sdf::ElementPtr jointElem = modelElem->GetElement("joint");
  EXPECT_TRUE(jointElem != nullptr);

  physics::JointState sdfJointState;
  sdfJointState.Load(jointElem);

  EXPECT_EQ(0u, defaultJointState.GetAngleCount());
  EXPECT_EQ(0u, sdfJointState.GetAngleCount());
}

TEST_F(JointStateTest, StreamSingleAxis)
{
  physics::JointState initialJointState;
  {
    std::ostringstream sdfStr;
    sdfStr << "<sdf version ='" << SDF_VERSION << "'>"
    << "<world name='default'>"
    << "<state world_name='default'>"
    << "<model name='model_00'>"
    << "  <joint name='joint_00'>"
    << "    <angle axis='0'>1.57</angle>"
    << "  </joint>"
    << "</model>"
    << "</state>"
    << "</world>"
    << "</sdf>";
    sdf::SDFPtr worldSDF(new sdf::SDF);
    worldSDF->SetFromString(sdfStr.str());
    EXPECT_TRUE(worldSDF->Root()->HasElement("world"));
    sdf::ElementPtr worldElem = worldSDF->Root()->GetElement("world");
    EXPECT_TRUE(worldElem->HasElement("state"));
    sdf::ElementPtr stateElem = worldElem->GetElement("state");
    EXPECT_TRUE(stateElem->HasElement("model"));
    sdf::ElementPtr modelElem = stateElem->GetElement("model");
    EXPECT_TRUE(modelElem != nullptr);
    sdf::ElementPtr jointElem = modelElem->GetElement("joint");
    EXPECT_TRUE(jointElem != nullptr);

    initialJointState.Load(jointElem);
  }
  ASSERT_EQ(1u, initialJointState.GetAngleCount());


  std::ostringstream sdfStr;
  sdfStr << "<sdf version ='" << SDF_VERSION << "'>"
  << "<world name='default'>"
  << "<state world_name='default'>"
  << "<model name='model_00'>"
  << initialJointState
  << "</model>"
  << "</state>"
  << "</world>"
  << "</sdf>";

  sdf::SDFPtr worldSDF(new sdf::SDF);
  worldSDF->SetFromString(sdfStr.str());
  EXPECT_TRUE(worldSDF->Root()->HasElement("world"));
  sdf::ElementPtr worldElem = worldSDF->Root()->GetElement("world");
  EXPECT_TRUE(worldElem->HasElement("state"));
  sdf::ElementPtr stateElem = worldElem->GetElement("state");
  EXPECT_TRUE(stateElem->HasElement("model"));
  sdf::ElementPtr modelElem = stateElem->GetElement("model");
  EXPECT_TRUE(modelElem != nullptr);
  sdf::ElementPtr jointElem = modelElem->GetElement("joint");
  EXPECT_TRUE(jointElem != nullptr);

  physics::JointState sdfJointState;
  sdfJointState.Load(jointElem);

  EXPECT_EQ(1u, sdfJointState.GetAngleCount());
  EXPECT_DOUBLE_EQ(1.57, sdfJointState.Position(0));
}

TEST_F(JointStateTest, StreamMultiAxis)
{
  physics::JointState initialJointState;
  {
    std::ostringstream sdfStr;
    sdfStr << "<sdf version ='" << SDF_VERSION << "'>"
    << "<world name='default'>"
    << "<state world_name='default'>"
    << "<model name='model_00'>"
    << "  <joint name='joint_00'>"
    << "    <angle axis='0'>1.57</angle>"
    << "    <angle axis='1'>3.14</angle>"
    << "  </joint>"
    << "</model>"
    << "</state>"
    << "</world>"
    << "</sdf>";
    sdf::SDFPtr worldSDF(new sdf::SDF);
    worldSDF->SetFromString(sdfStr.str());
    EXPECT_TRUE(worldSDF->Root()->HasElement("world"));
    sdf::ElementPtr worldElem = worldSDF->Root()->GetElement("world");
    EXPECT_TRUE(worldElem->HasElement("state"));
    sdf::ElementPtr stateElem = worldElem->GetElement("state");
    EXPECT_TRUE(stateElem->HasElement("model"));
    sdf::ElementPtr modelElem = stateElem->GetElement("model");
    EXPECT_TRUE(modelElem != nullptr);
    sdf::ElementPtr jointElem = modelElem->GetElement("joint");
    EXPECT_TRUE(jointElem != nullptr);

    initialJointState.Load(jointElem);
  }
  ASSERT_EQ(2u, initialJointState.GetAngleCount());


  std::ostringstream sdfStr;
  sdfStr << "<sdf version ='" << SDF_VERSION << "'>"
  << "<world name='default'>"
  << "<state world_name='default'>"
  << "<model name='model_00'>"
  << initialJointState
  << "</model>"
  << "</state>"
  << "</world>"
  << "</sdf>";

  sdf::SDFPtr worldSDF(new sdf::SDF);
  worldSDF->SetFromString(sdfStr.str());
  EXPECT_TRUE(worldSDF->Root()->HasElement("world"));
  sdf::ElementPtr worldElem = worldSDF->Root()->GetElement("world");
  EXPECT_TRUE(worldElem->HasElement("state"));
  sdf::ElementPtr stateElem = worldElem->GetElement("state");
  EXPECT_TRUE(stateElem->HasElement("model"));
  sdf::ElementPtr modelElem = stateElem->GetElement("model");
  EXPECT_TRUE(modelElem != nullptr);
  sdf::ElementPtr jointElem = modelElem->GetElement("joint");
  EXPECT_TRUE(jointElem != nullptr);

  physics::JointState sdfJointState;
  sdfJointState.Load(jointElem);

  EXPECT_EQ(2u, sdfJointState.GetAngleCount()) << sdfStr.str();
  EXPECT_DOUBLE_EQ(1.57, sdfJointState.Position(0));
  EXPECT_DOUBLE_EQ(3.14, sdfJointState.Position(1));
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
