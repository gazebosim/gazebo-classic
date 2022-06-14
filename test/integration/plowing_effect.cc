/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#include <cmath>
#include "gazebo/test/ServerFixture.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/sensors.hh"
#include "gazebo/common/common.hh"
#include "scans_cmp.h"

using namespace gazebo;

class PlowingEffect : public ServerFixture
{
 public: void RigidTerrain(const std::string &_physicsEngine);

  /// \brief CallbackRigidTerrain for subscribing to /gazebo/default/physics/contacts topic
 private: void CallbackRigidTerrain(const ConstContactsPtr &_msg);
};


////////////////////////////////////////////////////////////////////////
void PlowingEffect::CallbackRigidTerrain(const ConstContactsPtr &_msg)
{
  std::string collision1 = "original_tricycle::wheel_front::collision";
  std::string collision2 = "plowing_effect_ground_plane::link::collision";

  // no shift in contact point on rigid surface
  for(auto idx = 0; idx < _msg->contact_size(); ++idx)
  {
    const gazebo::msgs::Contact& contact = _msg->contact(idx);
    if(contact.collision1() == collision1 &&
       contact.collision2() == collision2)
    {
      const ignition::math::Vector3<double>& contact_point_normal =
                            ConvertIgn( contact.normal(0));
      const ignition::math::Vector3<double>& unit_normal =
                            ignition::math::Vector3<double>::UnitZ;
      double angle = acos(contact_point_normal.Dot(unit_normal)/
                     contact_point_normal.Length() * unit_normal.Length());
      ASSERT_EQ(angle, 0);
    }
  }
}

void PlowingEffect::RigidTerrain(const std::string &_physicsEngine)
{
  // Load an plowing effect world
  Load("worlds/plowing_effect_demo.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  const std::string& topic = "/gazebo/default/physics/contacts";
  std::list<std::string> topics =
      transport::getAdvertisedTopics("gazebo.msgs.Contacts");
  ASSERT_TRUE(topics.size() == 1);
  ASSERT_EQ(*topics.begin(), "/gazebo/default/physics/contacts");

  transport::SubscriberPtr sub = this->node->Subscribe(topic,
                                                       &PlowingEffect::CallbackRigidTerrain, this);

  world->Step(50);
}

TEST_F(PlowingEffect, RigidTerrain)
{
  RigidTerrain("ode");
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}