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
 public: void DeformableTerrain(const std::string &_physicsEngine);

  /// \brief CallbackRigidTerrain for subscribing to /gazebo/default/physics/contacts topic
 private: void CallbackRigidTerrain(const ConstContactsPtr &_msg);
 private: void CallbackDeformableTerrain(const ConstContactsPtr &_msg);

 private: physics::LinkPtr wheelLink = nullptr;
};

unsigned int g_messageCount = 0;

////////////////////////////////////////////////////////////////////////
void PlowingEffect::CallbackRigidTerrain(const ConstContactsPtr &_msg)
{
  std::string wheelCollisionStr = "original_tricycle::wheel_front::collision";
  std::string groundCollisionStr = "plowing_effect_ground_plane::link::collision";

  // no shift in contact point on rigid surface
  for(auto idx = 0; idx < _msg->contact_size(); ++idx)
  {
    const gazebo::msgs::Contact& contact = _msg->contact(idx);
    if(contact.collision1() == wheelCollisionStr &&
       contact.collision2() == groundCollisionStr)
    {
      const ignition::math::Vector3<double>& contactPointNormal =
                            ConvertIgn( contact.normal(0));
      const ignition::math::Vector3<double>& unitNormal =
                            ignition::math::Vector3<double>::UnitZ;
      double angle = acos(contactPointNormal.Dot(unitNormal)/
                     contactPointNormal.Length() * unitNormal.Length());
      ASSERT_EQ(angle, 0);
    }
  }
}

void PlowingEffect::CallbackDeformableTerrain(const ConstContactsPtr &_msg)
{
  std::string wheelCollisionStr = "my_tricycle::wheel_front::collision";
  std::string groundCollisionStr = "plowing_effect_ground_plane::link::collision";
  auto plowing_params = wheelLink->GetCollision("collision")->
                                  GetSDF()->GetElement("gz:plowing_wheel");

  auto maxDegrees = plowing_params->Get<double>("max_degrees");
  auto deadbandVelocity = plowing_params->Get<double>("deadband_velocity");
  auto saturationVelocity = plowing_params->Get<double>("saturation_velocity");

  ASSERT_EQ(maxDegrees, 15.0);
  ASSERT_EQ(deadbandVelocity, 0.5);
  ASSERT_EQ(saturationVelocity, 0.63);

  auto fdir1 = wheelLink->GetCollision("collision")->GetSDF()->
                          GetElement("surface")->GetElement("friction")->
                          GetElement("ode")->Get<ignition::math::Vector3<double>>("fdir1");

  for(auto idx = 0; idx < _msg->contact_size(); ++idx)
  {
    const gazebo::msgs::Contact& contact = _msg->contact(idx);
    if(contact.collision1() == wheelCollisionStr &&
        contact.collision2() == groundCollisionStr)
    {
      const ignition::math::Vector3<double>& contactPointNormal =
          ConvertIgn( contact.normal(0));
      const ignition::math::Vector3<double>& unitNormal =
          ignition::math::Vector3<double>::UnitZ;
      double angle = acos(contactPointNormal.Dot(unitNormal)/
                     contactPointNormal.Length() * unitNormal.Length());

      auto wheelLinearVelocity = this->wheelLink->GetCollision("collision")->WorldLinearVel();

      // Compute longitudinal unit vector as normal cross fdir1
      ignition::math::Vector3d unitLongitudinal =
                                contactPointNormal.Cross(fdir1);

      // Compute longitudinal speed (dot product)
      double wheelSpeedLongitudinal =
          wheelLinearVelocity.AbsDot(unitLongitudinal);

      // no shift in contact point
      if(wheelSpeedLongitudinal < deadbandVelocity)
      {
        EXPECT_EQ(angle, 0);
      }
    }
  }
}

void PlowingEffect::RigidTerrain(const std::string &_physicsEngine)
{
  // Load the plowing effect world
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
  world->Step(1);
}

void PlowingEffect::DeformableTerrain(const std::string &_physicsEngine)
{
  // Load the plowing effect world
  Load("worlds/plowing_effect_demo.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  const std::string& topic = "/gazebo/default/physics/contacts";
  std::list<std::string> topics =
      transport::getAdvertisedTopics("gazebo.msgs.Contacts");
  ASSERT_TRUE(topics.size() == 1);
  ASSERT_EQ(*topics.begin(), "/gazebo/default/physics/contacts");

  physics::ModelPtr model = world->ModelByName("my_tricycle");
  ASSERT_TRUE(model != nullptr);

  this->wheelLink = model->GetLink("wheel_front");
  ASSERT_TRUE(wheelLink != nullptr);

  transport::SubscriberPtr sub = this->node->Subscribe(topic,
                                                       &PlowingEffect::CallbackDeformableTerrain, this);
  world->Step(5000);
}

TEST_F(PlowingEffect, RigidTerrain)
{
  RigidTerrain("ode");
}

TEST_F(PlowingEffect, DeformableTerrain)
{
  DeformableTerrain("ode");
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}