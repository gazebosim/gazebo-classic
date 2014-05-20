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

#include <gtest/gtest.h>
#include "gazebo/physics/physics.hh"
// #include "gazebo/physics/Joint.hh"
// #include "gazebo/physics/ScrewJoint.hh"
#include "ServerFixture.hh"
#include "helper_physics_generator.hh"

using namespace gazebo;

class JointTestScrew : public ServerFixture,
                       public testing::WithParamInterface<const char*>
{
  /// \brief 
  /// \param[in] 
  public: void SetJointPositionTest(const std::string &_physicsEngine);
};

//////////////////////////////////////////////////
void JointTestScrew::SetJointPositionTest(const std::string &_physicsEngine)
{
  // init random seed
  srand(time(NULL));
  unsigned int seed = time(NULL);

  if (_physicsEngine == "bullet")
  {
    /// \TODO skipping bullet, see issue #1081
    gzerr << "BulletScrewJoint::GetAngle() is one step behind (issue #1081).\n";
    return;
  }

  if (_physicsEngine == "dart")
  {
    gzerr << "DART Screw Joint not yet implemented.\n";
    return;
  }

  // Load our screw joint test world
  Load("worlds/set_joint_position.world", true, _physicsEngine);

  // Get a pointer to the world, make sure world loads
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  physics->SetGravity(math::Vector3(0, 0, 0));

  // simulate 1 step
  world->Step(1);
  double t = world->GetSimTime().Double();

  // get time step size
  double dt = world->GetPhysicsEngine()->GetMaxStepSize();
  EXPECT_GT(dt, 0);
  gzlog << "dt : " << dt << "\n";

  // verify that time moves forward
  EXPECT_DOUBLE_EQ(t, dt);
  gzlog << "t after one step : " << t << "\n";

  // get pointer to model
  physics::ModelPtr model = world->GetModel("model_1");
  while (!model)
  {
    model = world->GetModel("model_1");
    gzdbg << "waiting for model_1 to spawn\n";
    sleep(1);
  }
  world->SetPaused(false);

  // intentionally break the joint using Link::SetWorldPose
  // let it conflict with Physics pose updates and make sure
  // internal model state stays consistent

  double start_time;
  double start_wall_time;
  double test_duration;
  double last_update_time;
  double elapsed_wall_time;

  physics::LinkPtr link_1 = model->GetLink("link_1");
  physics::LinkPtr link_2 = model->GetLink("link_2");
  physics::LinkPtr link_3 = model->GetLink("link_3");
  physics::LinkPtr link_4 = model->GetLink("link_4");
  physics::LinkPtr link_5 = model->GetLink("link_5");
  physics::LinkPtr link_2a = model->GetLink("link_2a");
  physics::LinkPtr link_2b = model->GetLink("link_2b");
  physics::LinkPtr link_3a = model->GetLink("link_3a");
  physics::LinkPtr link_4a = model->GetLink("link_4a");
  physics::LinkPtr link_5a = model->GetLink("link_5a");

  EXPECT_TRUE(link_1  != NULL);
  EXPECT_TRUE(link_2  != NULL);
  EXPECT_TRUE(link_3  != NULL);
  EXPECT_TRUE(link_4  != NULL);
  EXPECT_TRUE(link_5  != NULL);
  EXPECT_TRUE(link_2a != NULL);
  EXPECT_TRUE(link_2b != NULL);
  EXPECT_TRUE(link_3a != NULL);
  EXPECT_TRUE(link_4a != NULL);
  EXPECT_TRUE(link_5a != NULL);

  physics::JointPtr joint_01   = model->GetJoint("model_1::joint_01");
  physics::JointPtr joint_12   = model->GetJoint("model_1::joint_12");
  physics::JointPtr joint_23   = model->GetJoint("model_1::joint_23");
  physics::JointPtr joint_34   = model->GetJoint("model_1::joint_34");
  physics::JointPtr joint_45   = model->GetJoint("model_1::joint_45");
  physics::JointPtr joint_52   = model->GetJoint("model_1::joint_52");
  physics::JointPtr joint_22a  = model->GetJoint("model_1::joint_22a");
  physics::JointPtr joint_2a2b = model->GetJoint("model_1::joint_2a2b");
  physics::JointPtr joint_2b3a = model->GetJoint("model_1::joint_2b3a");
  physics::JointPtr joint_3a4a = model->GetJoint("model_1::joint_3a4a");
  physics::JointPtr joint_4a5a = model->GetJoint("model_1::joint_4a5a");
  physics::JointPtr joint_5a2b = model->GetJoint("model_1::joint_5a2b");

  EXPECT_TRUE(joint_01   != NULL);
  EXPECT_TRUE(joint_12   != NULL);
  EXPECT_TRUE(joint_23   != NULL);
  EXPECT_TRUE(joint_34   != NULL);
  EXPECT_TRUE(joint_45   != NULL);
  EXPECT_TRUE(joint_52   != NULL);
  EXPECT_TRUE(joint_22a  != NULL);
  EXPECT_TRUE(joint_2a2b != NULL);
  EXPECT_TRUE(joint_2b3a != NULL);
  EXPECT_TRUE(joint_3a4a != NULL);
  EXPECT_TRUE(joint_4a5a != NULL);
  EXPECT_TRUE(joint_5a2b != NULL);

  start_time = world->GetSimTime().Double();
  start_wall_time = world->GetRealTime().Double();
  test_duration = 10;
  gzdbg << " -------------------------------------------------------------\n";
  gzdbg << " Oscillate between two angles: -0.5 and 0.5 for "
        << test_duration << " secs\n";
  gzdbg << " see how well Joint::SetPosition deals with rapid updates.\n";
  gzdbg << " Publishing Joint::SetPosition every time step.\n";
  last_update_time = start_time;
  while (world->GetSimTime().Double() < start_time + test_duration)
  {
    last_update_time = world->GetSimTime().Double();
    // gzdbg << "setting link poses without violation\n";
    // double cur_time = world->GetSimTime().Double();
    joint_01->SetPosition(0, -0.5);
    joint_12->SetPosition(0, -0.5);
    joint_23->SetPosition(0, -0.5);
    joint_34->SetPosition(0, -0.5);
    joint_45->SetPosition(0, -0.5);
    joint_52->SetPosition(0, -0.5);
    joint_22a->SetPosition(0, -0.5);
    joint_2a2b->SetPosition(0, -0.5);
    joint_2b3a->SetPosition(0, -0.5);
    joint_3a4a->SetPosition(0, -0.5);
    joint_4a5a->SetPosition(0, -0.5);
    joint_5a2b->SetPosition(0, -0.5);

    joint_01->SetPosition(0, 0.5);
    joint_12->SetPosition(0, 0.5);
    joint_23->SetPosition(0, 0.5);
    joint_34->SetPosition(0, 0.5);
    joint_45->SetPosition(0, 0.5);
    joint_52->SetPosition(0, 0.5);
    joint_22a->SetPosition(0, 0.5);
    joint_2a2b->SetPosition(0, 0.5);
    joint_2b3a->SetPosition(0, 0.5);
    joint_3a4a->SetPosition(0, 0.5);
    joint_4a5a->SetPosition(0, 0.5);
    joint_5a2b->SetPosition(0, 0.5);

    EXPECT_EQ(link_1->GetWorldLinearVel(), math::Vector3(0, 0, 0));
    EXPECT_EQ(link_2->GetWorldLinearVel(), math::Vector3(0, 0, 0));
    EXPECT_EQ(link_3->GetWorldLinearVel(), math::Vector3(0, 0, 0));
    EXPECT_EQ(link_4->GetWorldLinearVel(), math::Vector3(0, 0, 0));
    EXPECT_EQ(link_5->GetWorldLinearVel(), math::Vector3(0, 0, 0));
    EXPECT_EQ(link_2a->GetWorldLinearVel(), math::Vector3(0, 0, 0));
    EXPECT_EQ(link_2b->GetWorldLinearVel(), math::Vector3(0, 0, 0));
    EXPECT_EQ(link_3a->GetWorldLinearVel(), math::Vector3(0, 0, 0));
    EXPECT_EQ(link_4a->GetWorldLinearVel(), math::Vector3(0, 0, 0));
    EXPECT_EQ(link_5a->GetWorldLinearVel(), math::Vector3(0, 0, 0));
    EXPECT_EQ(link_1->GetWorldAngularVel(), math::Vector3(0, 0, 0));
    EXPECT_EQ(link_2->GetWorldAngularVel(), math::Vector3(0, 0, 0));
    EXPECT_EQ(link_3->GetWorldAngularVel(), math::Vector3(0, 0, 0));
    EXPECT_EQ(link_4->GetWorldAngularVel(), math::Vector3(0, 0, 0));
    EXPECT_EQ(link_5->GetWorldAngularVel(), math::Vector3(0, 0, 0));
    EXPECT_EQ(link_2a->GetWorldAngularVel(), math::Vector3(0, 0, 0));
    EXPECT_EQ(link_2b->GetWorldAngularVel(), math::Vector3(0, 0, 0));
    EXPECT_EQ(link_3a->GetWorldAngularVel(), math::Vector3(0, 0, 0));
    EXPECT_EQ(link_4a->GetWorldAngularVel(), math::Vector3(0, 0, 0));
    EXPECT_EQ(link_5a->GetWorldAngularVel(), math::Vector3(0, 0, 0));
  }

  elapsed_wall_time = world->GetRealTime().Double() - start_wall_time;
  gzdbg << "  elapsed sim time [" << test_duration
    << "] elapsed wall time [" << elapsed_wall_time
    << "] sim performance [" << test_duration / elapsed_wall_time
    << "]\n";

  start_time = world->GetSimTime().Double();
  start_wall_time = world->GetRealTime().Double();
  test_duration = 20;
  const double pub_rate = 10.0;
  const double pause_time = 1.0;
  gzdbg << " -------------------------------------------------------------\n";
  gzdbg << " Send random joint position commands with " << pause_time
        << " sec delay between calls for " << test_duration
        << " secs, see how well Joint::SetPosition delas with random inputs.\n";
  gzdbg << " Publishing Joint::SetPosition at [" << pub_rate
        << "] Hz with real time duration.\n";
  last_update_time = start_wall_time;
  while (world->GetRealTime().Double() < start_wall_time + test_duration)
    if (world->GetRealTime().Double() - last_update_time >= (1.0/pub_rate))
    {
      last_update_time = world->GetRealTime().Double();
      joint_01->SetPosition(0,
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX));
      joint_12->SetPosition(0,
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX));
      joint_23->SetPosition(0,
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX));
      joint_34->SetPosition(0,
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX));
      joint_45->SetPosition(0,
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX));
      joint_52->SetPosition(0,
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX));
      joint_22a->SetPosition(0,
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX));
      joint_2a2b->SetPosition(0,
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX));
      joint_2b3a->SetPosition(0,
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX));
      joint_3a4a->SetPosition(0,
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX));
      joint_4a5a->SetPosition(0,
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX));
      joint_5a2b->SetPosition(0,
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX));
      sleep(pause_time);
      gzdbg << "running ["
            << world->GetRealTime().Double()
            << " / " << start_wall_time + test_duration
            << "]\n";

      EXPECT_EQ(link_1->GetWorldLinearVel(), math::Vector3(0, 0, 0));
      EXPECT_EQ(link_2->GetWorldLinearVel(), math::Vector3(0, 0, 0));
      EXPECT_EQ(link_3->GetWorldLinearVel(), math::Vector3(0, 0, 0));
      EXPECT_EQ(link_4->GetWorldLinearVel(), math::Vector3(0, 0, 0));
      EXPECT_EQ(link_5->GetWorldLinearVel(), math::Vector3(0, 0, 0));
      EXPECT_EQ(link_2a->GetWorldLinearVel(), math::Vector3(0, 0, 0));
      EXPECT_EQ(link_2b->GetWorldLinearVel(), math::Vector3(0, 0, 0));
      EXPECT_EQ(link_3a->GetWorldLinearVel(), math::Vector3(0, 0, 0));
      EXPECT_EQ(link_4a->GetWorldLinearVel(), math::Vector3(0, 0, 0));
      EXPECT_EQ(link_5a->GetWorldLinearVel(), math::Vector3(0, 0, 0));
      EXPECT_EQ(link_1->GetWorldAngularVel(), math::Vector3(0, 0, 0));
      EXPECT_EQ(link_2->GetWorldAngularVel(), math::Vector3(0, 0, 0));
      EXPECT_EQ(link_3->GetWorldAngularVel(), math::Vector3(0, 0, 0));
      EXPECT_EQ(link_4->GetWorldAngularVel(), math::Vector3(0, 0, 0));
      EXPECT_EQ(link_5->GetWorldAngularVel(), math::Vector3(0, 0, 0));
      EXPECT_EQ(link_2a->GetWorldAngularVel(), math::Vector3(0, 0, 0));
      EXPECT_EQ(link_2b->GetWorldAngularVel(), math::Vector3(0, 0, 0));
      EXPECT_EQ(link_3a->GetWorldAngularVel(), math::Vector3(0, 0, 0));
      EXPECT_EQ(link_4a->GetWorldAngularVel(), math::Vector3(0, 0, 0));
      EXPECT_EQ(link_5a->GetWorldAngularVel(), math::Vector3(0, 0, 0));
    }
  test_duration = world->GetSimTime().Double() - start_time;
  elapsed_wall_time = world->GetRealTime().Double() - start_wall_time;

  gzdbg << "  elapsed sim time [" << test_duration
    << "] elapsed wall time [" << elapsed_wall_time
    << "] sim performance [" << test_duration / elapsed_wall_time
    << "]\n";
}

TEST_P(JointTestScrew, SetJointPositionTest)
{
  SetJointPositionTest(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, JointTestScrew,
  PHYSICS_ENGINE_VALUES);

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
