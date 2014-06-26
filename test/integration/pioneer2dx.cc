/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#include "ServerFixture.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/common/Time.hh"
#include "helper_physics_generator.hh"

using namespace gazebo;
class Pioneer2dx : public ServerFixture,
                   public testing::WithParamInterface<const char*>
{
  public: void StraightLine(const std::string &_physicsEngine);
};

/////////////////////////////////////////////////
void Pioneer2dx::StraightLine(const std::string &_physicsEngine)
{
  if (_physicsEngine == "dart")
  {
    gzerr << "Abort test since dart does not handle pioneer2dx model yet.\n"
          << "Please see issue #912. "
          << "(https://bitbucket.org/osrf/gazebo/issue/912)\n";
    return;
  }

  Load("worlds/pioneer2dx.world", true, _physicsEngine);
  transport::PublisherPtr velPub = this->node->Advertise<gazebo::msgs::Pose>(
      "~/pioneer2dx/vel_cmd");

  std::string modelName = "pioneer2dx";

  // Check wheelSeparation and wheelRadius for DiffDrivePlugin
  {
    physics::WorldPtr world = physics::get_world();
    ASSERT_TRUE(world != NULL);

    physics::ModelPtr model = world->GetModel(modelName);
    ASSERT_TRUE(model != NULL);

    physics::JointPtr leftJoint = model->GetJoint("left_wheel_hinge");
    physics::JointPtr rightJoint = model->GetJoint("right_wheel_hinge");
    ASSERT_TRUE(leftJoint != NULL);
    ASSERT_TRUE(rightJoint != NULL);

    double wheelSeparation = leftJoint->GetAnchor(0).Distance(
                            rightJoint->GetAnchor(0));
    EXPECT_NEAR(0.28, wheelSeparation, 1e-3);

    physics::LinkPtr leftWheel = leftJoint->GetChild();
    // assume only one collision
    physics::CollisionPtr coll = leftWheel->GetCollisions()[0];

    double wheelRadius = 0;
    if (coll)
    {
      physics::ShapePtr shape = coll->GetShape();

      if (shape)
      {
        if (shape->HasType(gazebo::physics::Base::CYLINDER_SHAPE))
        {
          physics::CylinderShape *cyl =
              static_cast<physics::CylinderShape*>(shape.get());
          wheelRadius = cyl->GetRadius();
        }
        else if (shape->HasType(physics::Base::SPHERE_SHAPE))
        {
          physics::SphereShape *sph =
              static_cast<physics::SphereShape*>(shape.get());
          wheelRadius = sph->GetRadius();
        }
        else
          gzerr << "wheel shape is neither cylinder nor sphere,"
                << " what's radius here?\n";
      }
      else
        gzerr << "wheel collision GetShape failed\n";
    }
    else
      gzerr << "wheel link GetCollision failed\n";

    EXPECT_NEAR(0.11, wheelRadius, 1e-3);

    // Verify positive (and thus non-zero) value of wheelRadius
    // to prevent NaN's.
    ASSERT_GT(wheelRadius, 0.0);

    world->SetPaused(false);
  }

  gazebo::msgs::Pose msg;
  gazebo::msgs::Set(msg.mutable_position(),
      gazebo::math::Vector3(0.2, 0, 0));
  gazebo::msgs::Set(msg.mutable_orientation(),
      gazebo::math::Quaternion(0, 0, 0));
  velPub->Publish(msg);

  math::Pose startPose, endPose;
  startPose = this->poses[modelName];

  common::Time startTime = this->simTime;
  common::Time currTime = this->simTime;

  while (currTime - startTime < common::Time(20, 0))
  {
    common::Time::MSleep(100);
    currTime = this->simTime;
  }

  endPose = this->poses[modelName];

  double dist = (currTime - startTime).Double() * 0.2;
  std::cout << "Dist[" << dist << "]\n";
  std::cout << "EndPose.x[" << endPose.pos.x << "]\n";
  double tolerance = 0.1;
  if (_physicsEngine == "simbody")
  {
    // simbody results in a slower pioneer2dx, not sure why
    // see issue #866
    tolerance = 0.33;
  }
  EXPECT_LT(fabs(endPose.pos.x - dist), tolerance);
  EXPECT_LT(fabs(endPose.pos.y), 0.5);
  EXPECT_LT(fabs(endPose.pos.z), 0.01);
}


TEST_P(Pioneer2dx, StraightLine)
{
  StraightLine(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, Pioneer2dx, PHYSICS_ENGINE_VALUES);

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
