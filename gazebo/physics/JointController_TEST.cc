/*
 * Copyright (C) 2014-2016 Open Source Robotics Foundation
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

#include <gtest/gtest.h>

#include "gazebo/common/PID.hh"
#include "gazebo/math/Vector3.hh"
#include "gazebo/math/Angle.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/Joint.hh"
#include "gazebo/physics/JointController.hh"
#include "test/util.hh"

using namespace gazebo;

class JointControllerTest : public gazebo::testing::AutoLogFixture { };

class FakeJoint : public physics::Joint
{
  public: FakeJoint(physics::ModelPtr _model) : physics::Joint(_model)
          {}

  public: virtual physics::LinkPtr JointLink(const unsigned int) const
          {return physics::LinkPtr();}

  public: virtual bool AreConnected(physics::LinkPtr, physics::LinkPtr) const
          {return true;}

  public: virtual void SetAxis(const unsigned int,
              const ignition::math::Vector3d &)
          {}

  public: virtual void SetDamping(const unsigned int, const double)
          {}

  public: virtual void SetStiffnessDamping(const unsigned int, const double,
              const double , const double)
          {}

  public: virtual void SetStiffness(const unsigned int , const double)
          {}

  public: virtual ignition::math::Vector3d GlobalAxis(const unsigned int) const
          {return ignition::math::Vector3d::Zero;}

  public: virtual void SetAnchor(const unsigned int,
              const ignition::math::Vector3d &)
          {}

  public: virtual ignition::math::Vector3d Anchor(const unsigned int) const
          {return ignition::math::Vector3d::Zero;}

  public: virtual ignition::math::Angle HighStop(const unsigned int) const
          {return ignition::math::Angle::Zero;}

  public: virtual ignition::math::Angle LowStop(const unsigned int) const
          {return ignition::math::Angle::Zero;}

  public: virtual void SetVelocity(const unsigned int, const double)
          {}

  public: virtual double Velocity(const unsigned int) const
          {return 0.0;}

  public: virtual void SetForce(const unsigned int, const double)
          {}

  public: virtual physics::JointWrench ForceTorque(const unsigned int) const
          {return physics::JointWrench();}

  public: virtual unsigned int AngleCount() const
          {return 0;}

  public: virtual ignition::math::Vector3d LinkForce(const unsigned int) const
          {return ignition::math::Vector3d::Zero;}

  public: virtual ignition::math::Vector3d LinkTorque(const unsigned int) const
          {return ignition::math::Vector3d::Zero;}

  public: virtual void SetAttribute(const std::string &, const unsigned int,
              const boost::any &)
          {}

  public: virtual double GetAttribute(const std::string &,
              const unsigned int)
          {return 0.0;}

  public: virtual bool SetParam(const std::string &, const unsigned int,
              const boost::any &)
          {return true;}

  public: virtual double Param(const std::string &, const unsigned int)
          {return 0.0;}

  protected: virtual ignition::math::Angle AngleImpl(
                 const unsigned int) const
          {return ignition::math::Angle::Zero;}
};

/////////////////////////////////////////////////
TEST_F(JointControllerTest, Construction)
{
  // Create a dummy model
  physics::ModelPtr model(new physics::Model(physics::BasePtr()));
  EXPECT_TRUE(model != NULL);

  // Create the joint controller
  physics::JointControllerPtr jointController(
      new physics::JointController(model));
  EXPECT_TRUE(jointController != NULL);

  // All values should be empty
  EXPECT_TRUE(jointController->GetJoints().empty());
  EXPECT_TRUE(jointController->GetPositionPIDs().empty());
  EXPECT_TRUE(jointController->GetVelocityPIDs().empty());
  EXPECT_TRUE(jointController->GetForces().empty());
  EXPECT_TRUE(jointController->GetPositions().empty());
  EXPECT_TRUE(jointController->GetVelocities().empty());

  // The last update time should be zero.
  EXPECT_EQ(jointController->GetLastUpdateTime(), common::Time::Zero);
}

/////////////////////////////////////////////////
TEST_F(JointControllerTest, AddJoint)
{
  // Create a dummy model
  physics::ModelPtr model(new physics::Model(physics::BasePtr()));
  EXPECT_TRUE(model != NULL);

  // Create the joint controller
  physics::JointControllerPtr jointController(
      new physics::JointController(model));
  EXPECT_TRUE(jointController != NULL);

  physics::JointPtr joint(new FakeJoint(model));
  joint->SetName("joint");

  // There should be one joint in the controller
  jointController->AddJoint(joint);
  std::map<std::string, physics::JointPtr> joints =
    jointController->GetJoints();
  EXPECT_EQ(joints.size(), 1u);

  // Set a new position PID
  jointController->SetPositionPID(joint->ScopedName(), common::PID(4, 1, 9));

  // Check the new position PID values
  std::map<std::string, common::PID> posPids =
    jointController->GetPositionPIDs();
  EXPECT_EQ(posPids.size(), 1u);
  EXPECT_DOUBLE_EQ(posPids[joint->ScopedName()].GetPGain(), 4);
  EXPECT_DOUBLE_EQ(posPids[joint->ScopedName()].GetIGain(), 1);
  EXPECT_DOUBLE_EQ(posPids[joint->ScopedName()].GetDGain(), 9);

  // Restore the default position PID values
  jointController->SetPositionPID(
    joint->ScopedName(), common::PID(1, 0.1, 0.01));

  // Check the default position PID values
  posPids = jointController->GetPositionPIDs();
  EXPECT_EQ(posPids.size(), 1u);
  EXPECT_DOUBLE_EQ(posPids[joint->ScopedName()].GetPGain(), 1);
  EXPECT_DOUBLE_EQ(posPids[joint->ScopedName()].GetIGain(), 0.1);
  EXPECT_DOUBLE_EQ(posPids[joint->ScopedName()].GetDGain(), 0.01);

  // Set a new velocity PID
  jointController->SetVelocityPID(joint->ScopedName(), common::PID(4, 1, 9));

  // Check the new velocity PID values
  std::map<std::string, common::PID> velPids =
    jointController->GetVelocityPIDs();
  EXPECT_EQ(velPids.size(), 1u);
  EXPECT_DOUBLE_EQ(velPids[joint->ScopedName()].GetPGain(), 4);
  EXPECT_DOUBLE_EQ(velPids[joint->ScopedName()].GetIGain(), 1);
  EXPECT_DOUBLE_EQ(velPids[joint->ScopedName()].GetDGain(), 9);

  // Restore the default velocity PID values
  jointController->SetVelocityPID(
    joint->ScopedName(), common::PID(1, 0.1, 0.01));

  // Check the default velocity PID values
  velPids = jointController->GetVelocityPIDs();
  EXPECT_EQ(velPids.size(), 1u);
  EXPECT_DOUBLE_EQ(velPids[joint->ScopedName()].GetPGain(), 1);
  EXPECT_DOUBLE_EQ(velPids[joint->ScopedName()].GetIGain(), 0.1);
  EXPECT_DOUBLE_EQ(velPids[joint->ScopedName()].GetDGain(), 0.01);

  // Set a joint position target
  EXPECT_TRUE(jointController->SetPositionTarget(
        joint->ScopedName(), 12.3));
  std::map<std::string, double> positions = jointController->GetPositions();
  EXPECT_EQ(positions.size(), 1u);
  EXPECT_DOUBLE_EQ(positions[joint->ScopedName()], 12.3);

  // Set a joint velocity target
  EXPECT_TRUE(jointController->SetVelocityTarget(
        joint->ScopedName(), 3.21));
  std::map<std::string, double> velocities = jointController->GetVelocities();
  EXPECT_EQ(velocities.size(), 1u);
  EXPECT_DOUBLE_EQ(velocities[joint->ScopedName()], 3.21);

  // Try setting a position target on a joint that doesn't exist.
  EXPECT_FALSE(jointController->SetPositionTarget("my_bad_name", 12.3));
  positions = jointController->GetPositions();
  EXPECT_EQ(positions.size(), 1u);
  EXPECT_DOUBLE_EQ(positions[joint->ScopedName()], 12.3);

  // Try setting a velocity target on a joint that doesn't exist.
  EXPECT_FALSE(jointController->SetVelocityTarget("my_bad_name", 3.21));
  velocities = jointController->GetVelocities();
  EXPECT_EQ(velocities.size(), 1u);
  EXPECT_DOUBLE_EQ(velocities[joint->ScopedName()], 3.21);

  // Reset the controller
  jointController->Reset();
  positions = jointController->GetPositions();
  velocities = jointController->GetVelocities();
  EXPECT_EQ(positions.size(), 0u);
  EXPECT_EQ(velocities.size(), 0u);
}

/////////////////////////////////////////////////
TEST_F(JointControllerTest, SetJointPositions)
{
  // Create a dummy model
  physics::ModelPtr model(new physics::Model(physics::BasePtr()));
  EXPECT_TRUE(model != NULL);

  // Create the joint controller
  physics::JointControllerPtr jointController(
      new physics::JointController(model));
  EXPECT_TRUE(jointController != NULL);

  physics::JointPtr joint1(new FakeJoint(model));
  joint1->SetName("joint1");

  physics::JointPtr joint2(new FakeJoint(model));
  joint2->SetName("joint2");

  // Add the joints.
  jointController->AddJoint(joint1);
  jointController->AddJoint(joint2);
  std::map<std::string, physics::JointPtr> joints =
    jointController->GetJoints();
  EXPECT_EQ(joints.size(), 2u);

  // Set joint positions for the two joints, and expect no expections.
  std::map<std::string, double> positions;
  positions[joint1->ScopedName()] = 1.2;
  positions[joint2->ScopedName()] = 2.3;
  EXPECT_NO_THROW(jointController->SetJointPositions(positions));
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
