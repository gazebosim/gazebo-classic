/*
 * Copyright (C) 2013-2015 Open Source Robotics Foundation
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
#include "gazebo/physics/physics.hh"
#include "gazebo/test/helper_physics_generator.hh"

using namespace gazebo;

class Issue624Test : public ServerFixture,
                     public testing::WithParamInterface<const char*>
{
  public: void CollisionWorldPose(const std::string &_physicsEngine);
};


/////////////////////////////////////////////////
// \brief Test for issue #624
void Issue624Test::CollisionWorldPose(const std::string &_physicsEngine)
{
  // Load an empty world
  Load("worlds/empty.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  // add a link with a collision body in it

  // Spawn some custom model
  {
    msgs::Factory msg;
    std::ostringstream newModelStr;

    std::string name = "box_1";

    newModelStr << "<sdf version='" << SDF_VERSION << "'>"
      << "<model name ='" << name << "'>"
      << "<static>false</static>"
      << "<pose>1 2 3 0.5 0.5 0.5</pose>"
      << "<link name ='body'>"
      << "  <pose>2 3 4 0.6 0.6 0.6</pose>"
      << "  <collision name ='col1'>"
      << "    <pose>3 4 5 0.7 0.7 0.7</pose>"
      << "    <geometry>"
      << "      <box><size>1 1 1</size></box>"
      << "    </geometry>"
      << "  </collision>"
      << "  <visual name ='vis1'>"
      << "    <pose>3 4 5 0.7 0.7 0.7</pose>"
      << "    <geometry>"
      << "      <box><size>1 1 1</size></box>"
      << "    </geometry>"
      << "  </visual>"
      << "  <collision name ='col2'>"
      << "    <pose>6 7 8 0.8 0.8 0.8</pose>"
      << "    <geometry>"
      << "      <box><size>1 1 1</size></box>"
      << "    </geometry>"
      << "  </collision>"
      << "  <visual name ='vis2'>"
      << "    <pose>6 7 8 0.8 0.8 0.8</pose>"
      << "    <geometry>"
      << "      <box><size>1 1 1</size></box>"
      << "    </geometry>"
      << "  </visual>"
      << "</link>"
      << "</model>"
      << "</sdf>";

    msg.set_sdf(newModelStr.str());
    this->factoryPub->Publish(msg);

    // Wait for the entity to spawn
    while (!this->HasEntity(name))
      common::Time::MSleep(100);
  }
  physics::ModelPtr model = world->GetModel("box_1");

  physics::Link_V links = model->GetLinks();
  for (physics::Link_V::iterator li = links.begin(); li != links.end(); ++li)
  {
    physics::LinkPtr link = *li;
    physics::Collision_V collisions = link->GetCollisions();
    for (physics::Collision_V::iterator ci = collisions.begin();
       ci != collisions.end(); ++ci)
    {
      gzdbg << "name [" << (*ci)->GetName()
            << "] abs pose [" << (*ci)->GetWorldPose()
            << "] rel pose [" << (*ci)->GetRelativePose() << "]\n";
      if ((*ci)->GetName() == "col1")
      {
        EXPECT_EQ((*ci)->GetWorldPose(),
          math::Pose(3, 4, 5, 0.7, 0.7, 0.7) +
          math::Pose(2, 3, 4, 0.6, 0.6, 0.6) +
          math::Pose(1, 2, 3, 0.5, 0.5, 0.5));
        EXPECT_EQ((*ci)->GetRelativePose(),
          math::Pose(3, 4, 5, 0.7, 0.7, 0.7));
      }
      else if ((*ci)->GetName() == "col2")
      {
        EXPECT_EQ((*ci)->GetWorldPose(),
          math::Pose(6, 7, 8, 0.8, 0.8, 0.8) +
          math::Pose(2, 3, 4, 0.6, 0.6, 0.6) +
          math::Pose(1, 2, 3, 0.5, 0.5, 0.5));
        EXPECT_EQ((*ci)->GetRelativePose(),
          math::Pose(6, 7, 8, 0.8, 0.8, 0.8));
      }
    }
  }
}

TEST_P(Issue624Test, CollisionWorldPose)
{
  CollisionWorldPose(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, Issue624Test, PHYSICS_ENGINE_VALUES);

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
