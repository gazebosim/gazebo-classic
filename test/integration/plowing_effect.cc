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

class PlowingEffectTricycle : public ServerFixture
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
 private: double plowingAngleRigidTerrain_;

 private: const int maxCallbackCountRigidTerrain_ = 200;
 private: bool havePrintedNoPlowingEffect_ = false;
 private: bool havePrintedLinearPlowingEffect_ = false;
 private: bool havePrintedMaxPlowingEffect_ = false;

  /// constant params from SDF
 private: double maxRadians_;
 private: double deadbandVelocity_;
 private: double saturationVelocity_;
 private: ignition::math::Vector3<double> fdir1_;

};

/** plowing effect on four spheres of same size and inertia
 * Plowing Effect Sequence:
 * modelSphere4 > modelSphere3 > modelSphere2 > modelSphere1
 * where modelSphere1 has zero plowing effect
 */
class PlowingEffectSpheres : public ServerFixture
{
  /// \brief test for max plowing angle at saturation velocity
 public: void MaxPlowingAngle(const std::string &_physicsEngine);
 private: void CallbackMaxPlowingAngle(const ConstContactsPtr &_msg);

 private: physics::CollisionPtr sphere1CollisionPtr_ = nullptr;
 private: physics::CollisionPtr sphere2CollisionPtr_ = nullptr;
 private: physics::CollisionPtr sphere3CollisionPtr_ = nullptr;
 private: physics::CollisionPtr sphere4CollisionPtr_ = nullptr;

 private: int callbackCount_ = 1;
 private: double sphere1PlowingAngle_;
 private: double sphere2PlowingAngle_;
 private: double sphere3PlowingAngle_;
 private: double sphere4PlowingAngle_;
};

////////////////////////////////////////////////////////////////////////
void PlowingEffectTricycle::CallbackRigidTerrain(const ConstContactsPtr &_msg)
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
      plowingAngleRigidTerrain_ = acos(contactPointNormal.Dot(unitNormal)/
                            contactPointNormal.Length() * unitNormal.Length());
    }
  }
}

void PlowingEffectTricycle::CallbackDeformableTerrain(const ConstContactsPtr &_msg)
{
  std::string wheelCollisionStr = "my_tricycle::wheel_rear_left::collision";
  std::string groundCollisionStr = "plowing_effect_ground_plane::link::collision";

  bool contactFound = false;
  for(auto idx = 0; idx < _msg->contact_size(); ++idx)
  {
    const gazebo::msgs::Contact& contact = _msg->contact(idx);
    if(contact.collision1() == wheelCollisionStr &&
        contact.collision2() == groundCollisionStr)
    {
      contactFound = true;
      const auto& contactPointNormal = ConvertIgn( contact.normal(0));
      const auto& unitNormal = ignition::math::Vector3<double>::UnitZ;

      // compute longitudinal unit vector as normal cross fdir1_
      const auto& wheelLinearVelocity = wheelCollisionPtr_->WorldLinearVel();
      const auto& unitLongitudinal = fdir1_.Cross(unitNormal);
      double wheelSpeedLongitudinal = wheelLinearVelocity.Dot(unitLongitudinal);
      
      // compute the angle contact point makes with unit normal
      double angle = acos(contactPointNormal.Dot(unitNormal) /
                      contactPointNormal.Length());

      // check that longitudinal velocity and contact normal point in
      // opposite directions
      EXPECT_LE(unitLongitudinal.Dot(contactPointNormal), 0);

      // no plowing effect
      if(wheelSpeedLongitudinal <= deadbandVelocity_)
      {
        if (!havePrintedNoPlowingEffect_)
        {
          havePrintedNoPlowingEffect_ = true;
          gzdbg << "No plowing effect" << "\n";
        }
        EXPECT_EQ(angle, 0);
      }

      // linear plowing effect
      else if(wheelSpeedLongitudinal > deadbandVelocity_ &&
              wheelSpeedLongitudinal < saturationVelocity_)
      {
        if (!havePrintedLinearPlowingEffect_)
        {
          havePrintedLinearPlowingEffect_ = true;
          gzdbg << "Reached linear plowing effect" << "\n";
        }
        ASSERT_LT(angle, maxRadians_);
        ASSERT_GE(angle, 0);
      }

      // maximum plowing effect.
      else if(wheelSpeedLongitudinal >= saturationVelocity_)
      {
        if (!havePrintedMaxPlowingEffect_)
        {
          havePrintedMaxPlowingEffect_ = true;
          gzdbg << "Reached maximum plowing effect" << "\n";
        }
        EXPECT_NEAR(angle, maxRadians_, 1e-6);
      }
    }
  }
  EXPECT_TRUE(contactFound);
}

void PlowingEffectSpheres::CallbackMaxPlowingAngle(const ConstContactsPtr &_msg)
{
  if(callbackCount_ < 2300)
  {
    callbackCount_ = callbackCount_ + 1;
    return;
  }

  std::string sphere1CollisionStr = "sphere1::base_link::collision";
  std::string sphere2CollisionStr = "sphere2::base_link::collision";
  std::string sphere3CollisionStr = "sphere3::base_link::collision";
  std::string sphere4CollisionStr = "sphere4::base_link::collision";
  std::string groundCollisionStr = "plowing_effect_ground_plane::link::collision";

  for(auto idx = 0; idx < _msg->contact_size(); ++idx)
  {
    const gazebo::msgs::Contact& contact = _msg->contact(idx);
    if(contact.collision2() == groundCollisionStr)
    {
      const auto& contactPointNormal = ConvertIgn( contact.normal(0));
      const auto& unitNormal = ignition::math::Vector3<double>::UnitZ;

      double angle = acos(contactPointNormal.Dot(unitNormal)/
          contactPointNormal.Length() * unitNormal.Length());

      if(contact.collision1() == sphere1CollisionStr)
      {
        sphere1PlowingAngle_ = angle;
      }
      else if(contact.collision1() == sphere2CollisionStr)
      {
        sphere2PlowingAngle_ = angle;
      }
      else if(contact.collision1() == sphere3CollisionStr)
      {
        sphere3PlowingAngle_ = angle;
      }
      else if(contact.collision1() == sphere4CollisionStr)
      {
        sphere4PlowingAngle_ = angle;
      }
    }
  }
}

void PlowingEffectTricycle::RigidTerrain(const std::string &_physicsEngine)
{
  gazebo::common::SystemPaths::Instance()->AddModelPaths(
                    PROJECT_SOURCE_PATH "/test/models");
  // Load the plowing effect world
  Load("worlds/plowing_effect_tricycle_demo.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  const std::string& contactsTopic = "/gazebo/default/physics/contacts";
  std::list<std::string> topics =
      transport::getAdvertisedTopics("gazebo.msgs.Contacts");
  ASSERT_TRUE(topics.size() == 1);
  ASSERT_EQ(*topics.begin(), "/gazebo/default/physics/contacts");

  transport::SubscriberPtr sub = this->node->Subscribe(contactsTopic,
                                                       &PlowingEffectTricycle::CallbackRigidTerrain, this);
  world->Step(this->maxCallbackCountRigidTerrain_);
  ASSERT_EQ(this->plowingAngleRigidTerrain_, 0);
}

void PlowingEffectTricycle::DeformableTerrain(const std::string &_physicsEngine)
{
  gazebo::common::SystemPaths::Instance()->AddModelPaths(
                    PROJECT_SOURCE_PATH "/test/models");
  // Load the plowing effect world
  Load("worlds/plowing_effect_tricycle_demo.world", true, _physicsEngine);
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

  this->wheelCollisionPtr_ = wheelLinkPtr->GetCollision("collision");
  ASSERT_TRUE(this->wheelCollisionPtr_ != nullptr);

  auto plowingParams = this->wheelCollisionPtr_->GetSDF()->
                                  GetElement("gz:plowing_wheel");
  ASSERT_TRUE(plowingParams != nullptr);

  ignition::math::Angle maxAngle;
  auto maxDegree = plowingParams->Get<double>("max_degrees");
  maxAngle.SetDegree(maxDegree);

  this->maxRadians_ = maxAngle.Radian();
  this->deadbandVelocity_ = plowingParams->Get<double>("deadband_velocity");
  this->saturationVelocity_ = plowingParams->Get<double>("saturation_velocity");

  ASSERT_EQ(maxDegree, 5);
  ASSERT_EQ(this->deadbandVelocity_, 0.3);
  ASSERT_EQ(this->saturationVelocity_, 0.4);

  this->fdir1_ = wheelCollisionPtr_->GetSDF()->GetElement("surface")->
                 GetElement("friction")->GetElement("ode")->
                 Get<ignition::math::Vector3<double>>("fdir1");
  ASSERT_EQ(this->fdir1_, ignition::math::Vector3<double>(0, 1, 0));

  transport::SubscriberPtr sub = this->node->Subscribe(contactsTopic,
                              &PlowingEffectTricycle::CallbackDeformableTerrain, this);
  world->Step(500);
}

void PlowingEffectSpheres::MaxPlowingAngle(const std::string &_physicsEngine)
{
  // Load the plowing effect world
  Load("worlds/plowing_effect_spheres_test.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  const std::string& contactsTopic = "/gazebo/default/physics/contacts";
  std::list<std::string> topics =
      transport::getAdvertisedTopics("gazebo.msgs.Contacts");
  ASSERT_TRUE(topics.size() == 1);
  ASSERT_EQ(*topics.begin(), "/gazebo/default/physics/contacts");

  physics::ModelPtr modelSphere1 = world->ModelByName("sphere1");
  physics::ModelPtr modelSphere2 = world->ModelByName("sphere2");
  physics::ModelPtr modelSphere3 = world->ModelByName("sphere3");
  physics::ModelPtr modelSphere4 = world->ModelByName("sphere4");
  ASSERT_TRUE(modelSphere1 != nullptr);
  ASSERT_TRUE(modelSphere2 != nullptr);
  ASSERT_TRUE(modelSphere3 != nullptr);
  ASSERT_TRUE(modelSphere4 != nullptr);

  physics::LinkPtr sphere1LinkPtr = modelSphere1->GetLink("base_link");
  physics::LinkPtr sphere2LinkPtr = modelSphere2->GetLink("base_link");
  physics::LinkPtr sphere3LinkPtr = modelSphere3->GetLink("base_link");
  physics::LinkPtr sphere4LinkPtr = modelSphere4->GetLink("base_link");
  ASSERT_TRUE(sphere1LinkPtr != nullptr);
  ASSERT_TRUE(sphere2LinkPtr != nullptr);
  ASSERT_TRUE(sphere3LinkPtr != nullptr);
  ASSERT_TRUE(sphere4LinkPtr != nullptr);

  this->sphere1CollisionPtr_ = sphere1LinkPtr->GetCollision("collision");
  this->sphere2CollisionPtr_ = sphere2LinkPtr->GetCollision("collision");
  this->sphere3CollisionPtr_ = sphere3LinkPtr->GetCollision("collision");
  this->sphere4CollisionPtr_ = sphere4LinkPtr->GetCollision("collision");
  ASSERT_TRUE(sphere1CollisionPtr_ != nullptr);
  ASSERT_TRUE(sphere2CollisionPtr_ != nullptr);
  ASSERT_TRUE(sphere3CollisionPtr_ != nullptr);
  ASSERT_TRUE(sphere4CollisionPtr_ != nullptr);

  auto sphere2PlowingParams = sphere2CollisionPtr_->GetSDF()->
                                        GetElement("gz:plowing_wheel");
  auto sphere3PlowingParams = sphere3CollisionPtr_->GetSDF()->
                                        GetElement("gz:plowing_wheel");
  auto sphere4PlowingParams = sphere4CollisionPtr_->GetSDF()->
                                        GetElement("gz:plowing_wheel");
  ASSERT_TRUE(this->sphere2CollisionPtr_ != nullptr);
  ASSERT_TRUE(this->sphere3CollisionPtr_ != nullptr);
  ASSERT_TRUE(this->sphere4CollisionPtr_ != nullptr);

  // zero plowing effect for sphere1
  auto sphere2MaxDegree = sphere2PlowingParams->Get<double>("max_degrees");
  auto sphere3MaxDegree = sphere3PlowingParams->Get<double>("max_degrees");
  auto sphere4MaxDegree = sphere4PlowingParams->Get<double>("max_degrees");

  ignition::math::Angle sphere2MaxAngle;
  ignition::math::Angle sphere3MaxAngle;
  ignition::math::Angle sphere4MaxAngle;

  sphere2MaxAngle.SetDegree(sphere2MaxDegree);
  sphere3MaxAngle.SetDegree(sphere3MaxDegree);
  sphere4MaxAngle.SetDegree(sphere4MaxDegree);

  auto sphere2MaxRadians = sphere2MaxAngle.Radian();
  auto sphere3MaxRadians = sphere3MaxAngle.Radian();
  auto sphere4MaxRadians = sphere4MaxAngle.Radian();

  transport::SubscriberPtr sub = this->node->Subscribe(contactsTopic,
                                       &PlowingEffectSpheres::CallbackMaxPlowingAngle, this);

  world->Step(2500);

  double error = 1e-14;
  // verify spheres have reached max plowing angle
  EXPECT_EQ(this->sphere1PlowingAngle_, 0); // no plowing effect
  EXPECT_NEAR(this->sphere2PlowingAngle_, sphere2MaxRadians, error);
  EXPECT_NEAR(this->sphere3PlowingAngle_, sphere3MaxRadians, error);
  EXPECT_NEAR(this->sphere4PlowingAngle_, sphere4MaxRadians, error);
}

TEST_F(PlowingEffectTricycle, RigidTerrain)
{
  RigidTerrain("ode");
}

TEST_F(PlowingEffectTricycle, DeformableTerrain)
{
  DeformableTerrain("ode");
}

TEST_F(PlowingEffectSpheres, MaxPlowingAngle)
{
  MaxPlowingAngle("ode");
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}