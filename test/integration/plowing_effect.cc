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
  /// \brief unit test for RigidTerrain
 public: void RigidTerrain(const std::string &_physicsEngine);

  /// \brief unit test for DeformableTerrain
 public: void DeformableTerrain(const std::string &_physicsEngine);

  /// \brief CallbackRigidTerrain && CallbackDeformableTerrain for
  /// subscribing to /gazebo/default/physics/contacts topic
 private: void CallbackRigidTerrain(const ConstContactsPtr &_msg);
 private: void CallbackDeformableTerrain(const ConstContactsPtr &_msg);

 private: physics::CollisionPtr wheelCollisionPtr_ = nullptr;

  /// constant params from SDF
 private: double maxRadians_;
 private: double deadbandVelocity_;
 private: double saturationVelocity_;
 private: ignition::math::Vector3<double> fdir1_;
};

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

  for(auto idx = 0; idx < _msg->contact_size(); ++idx)
  {
    const gazebo::msgs::Contact& contact = _msg->contact(idx);
    if(contact.collision1() == wheelCollisionStr &&
        contact.collision2() == groundCollisionStr)
    {
      const auto& contactPointNormal = ConvertIgn( contact.normal(0));
      const auto& unitNormal = ignition::math::Vector3<double>::UnitZ;

      // compute longitudinal unit vector as normal cross fdir1_
      const auto& wheelLinearVelocity = wheelCollisionPtr_->WorldLinearVel();
      const auto& unitLongitudinal = contactPointNormal.Cross(fdir1_);
      double wheelSpeedLongitudinal = wheelLinearVelocity.AbsDot(unitLongitudinal);
      
      // compute the angle contact point makes with unit normal
      double angle = acos(contactPointNormal.Dot(unitNormal)/
                      contactPointNormal.Length() * unitNormal.Length());

      // no plowing effect
      if(wheelSpeedLongitudinal < deadbandVelocity_)
      {
        EXPECT_EQ(angle, 0);
      }

      // plowing effect
      else if(wheelSpeedLongitudinal > deadbandVelocity_ &&
              wheelSpeedLongitudinal < saturationVelocity_)
      {
        ASSERT_LT(angle, maxRadians_);
        ASSERT_GT(angle, 0);
      }

      // maximum plowing effect.
      else if(wheelSpeedLongitudinal >= saturationVelocity_)
      {
        gzdbg << "Reached maximum plowing effect" << "\n";
        EXPECT_EQ(angle, maxRadians_);
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

  const std::string& contactsTopic = "/gazebo/default/physics/contacts";
  std::list<std::string> topics =
      transport::getAdvertisedTopics("gazebo.msgs.Contacts");
  ASSERT_TRUE(topics.size() == 1);
  ASSERT_EQ(*topics.begin(), "/gazebo/default/physics/contacts");

  transport::SubscriberPtr sub = this->node->Subscribe(contactsTopic,
                                 &PlowingEffect::CallbackRigidTerrain, this);
  world->Step(100);
}

void PlowingEffect::DeformableTerrain(const std::string &_physicsEngine)
{
  // Load the plowing effect world
  Load("worlds/plowing_effect_demo.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  const std::string& contactsTopic = "/gazebo/default/physics/contacts";
  std::list<std::string> topics =
      transport::getAdvertisedTopics("gazebo.msgs.Contacts");
  ASSERT_TRUE(topics.size() == 1);
  ASSERT_EQ(*topics.begin(), "/gazebo/default/physics/contacts");

  physics::ModelPtr model = world->ModelByName("my_tricycle");
  ASSERT_TRUE(model != nullptr);

  physics::LinkPtr wheelLinkPtr = model->GetLink("wheel_front");
  ASSERT_TRUE(wheelLinkPtr != nullptr);

  wheelCollisionPtr_ = wheelLinkPtr->GetCollision("collision");
  ASSERT_TRUE(wheelCollisionPtr_ != nullptr);

  auto plowing_params = wheelCollisionPtr_->GetSDF()->GetElement("gz:plowing_wheel");
  
  ignition::math::Angle maxAngle;
  auto maxDegree = plowing_params->Get<double>("max_degrees");
  maxAngle.SetDegree(maxDegree);
  
  maxRadians_ = maxAngle.Radian();
  deadbandVelocity_ = plowing_params->Get<double>("deadband_velocity");
  saturationVelocity_ = plowing_params->Get<double>("saturation_velocity");

  ASSERT_EQ(maxDegree, 15);
  ASSERT_EQ(deadbandVelocity_, 0.5);
  ASSERT_EQ(saturationVelocity_, 0.63);

  fdir1_ =  wheelCollisionPtr_->GetSDF()->GetElement("surface")->
                 GetElement("friction")->GetElement("ode")->
                 Get<ignition::math::Vector3<double>>("fdir1");
  ASSERT_EQ(fdir1_, ignition::math::Vector3<double>(0, 1, 0));

  transport::SubscriberPtr sub = this->node->Subscribe(contactsTopic,
                                                       &PlowingEffect::CallbackDeformableTerrain, this);
  world->Step(100);
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