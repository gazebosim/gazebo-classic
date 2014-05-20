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

#define TOL 0.001
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
  const double pause_time = 1.0;

  physics::Link_V links;
  links.push_back(model->GetLink("link_1"));
  links.push_back(model->GetLink("link_2"));
  links.push_back(model->GetLink("link_3"));
  links.push_back(model->GetLink("link_4"));
  links.push_back(model->GetLink("link_5"));
  links.push_back(model->GetLink("link_2a"));
  links.push_back(model->GetLink("link_2b"));
  links.push_back(model->GetLink("link_3a"));
  links.push_back(model->GetLink("link_4a"));
  links.push_back(model->GetLink("link_5a"));

  for (physics::Link_V::iterator li = links.begin(); li != links.end(); ++li)
  {
    EXPECT_TRUE((*li)  != NULL);
  }

  physics::Joint_V joints;
  joints.push_back(model->GetJoint("model_1::joint_01"));
  joints.push_back(model->GetJoint("model_1::joint_12"));
  joints.push_back(model->GetJoint("model_1::joint_23"));
  joints.push_back(model->GetJoint("model_1::joint_34"));
  joints.push_back(model->GetJoint("model_1::joint_45"));
  joints.push_back(model->GetJoint("model_1::joint_52"));
  joints.push_back(model->GetJoint("model_1::joint_22a"));
  joints.push_back(model->GetJoint("model_1::joint_2a2b"));
  joints.push_back(model->GetJoint("model_1::joint_2b3a"));
  joints.push_back(model->GetJoint("model_1::joint_3a4a"));
  joints.push_back(model->GetJoint("model_1::joint_4a5a"));
  joints.push_back(model->GetJoint("model_1::joint_5a2b"));

  for (physics::Joint_V::iterator ji = joints.begin(); ji != joints.end(); ++ji)
  {
    EXPECT_TRUE((*ji)  != NULL);
  }

  start_time = world->GetSimTime().Double();
  start_wall_time = world->GetRealTime().Double();
  test_duration = 10;
  gzdbg << " -------------------------------------------------------------\n";
  gzdbg << " Oscillate between two angles: Setting joint positions -0.5 and 0.5"
        << " with " << pause_time
        << " sec delay between calls for " << test_duration << " secs\n";
  gzdbg << " see how well Joint::SetPosition deals with rapid updates.\n";
  gzdbg << " Publishing Joint::SetPosition every time step.\n";
  last_update_time = start_time;
  bool flip = true;
  // debug:
  // getchar();
  while (world->GetSimTime().Double() < start_time + test_duration)
  {
    last_update_time = world->GetSimTime().Double();
    // gzdbg << "setting link poses without violation\n";
    // double cur_time = world->GetSimTime().Double();
    for (physics::Joint_V::iterator ji = joints.begin();
                                    ji != joints.end(); ++ji)
    {
      if (flip)
        (*ji)->SetPosition(0, 0.5);
      else
        (*ji)->SetPosition(0, -0.5);
    }
    flip = !flip;

    sleep(pause_time);

    for (physics::Link_V::iterator li = links.begin();
                                   li != links.end(); ++li)
    {
      math::Vector3 linVel = (*li)->GetWorldLinearVel();
      math::Vector3 angVel = (*li)->GetWorldAngularVel();
      EXPECT_NEAR(linVel.x, 0, TOL);
      EXPECT_NEAR(linVel.y, 0, TOL);
      EXPECT_NEAR(linVel.z, 0, TOL);
      EXPECT_NEAR(angVel.x, 0, TOL);
      EXPECT_NEAR(angVel.y, 0, TOL);
      EXPECT_NEAR(angVel.z, 0, TOL);

      // debug:
      // if (abs(linVel.x) > 0 ||
      //     abs(linVel.y) > 0 ||
      //     abs(linVel.z) > 0)
      // {
      //   // world->EnablePhysicsEngine(false);
      //   world->SetPaused(true);
      //   gzerr << "stop";
      //   getchar();
      // }
    }
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
      for (physics::Joint_V::iterator ji = joints.begin();
                                      ji != joints.end(); ++ji)
      {
        (*ji)->SetPosition(0,
            static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX));
      }

      gzdbg << "running ["
            << world->GetRealTime().Double()
            << " / " << start_wall_time + test_duration
            << "]\n";

      sleep(pause_time);

      for (physics::Link_V::iterator li = links.begin();
                                     li != links.end(); ++li)
      {
        math::Vector3 linVel = (*li)->GetWorldLinearVel();
        math::Vector3 angVel = (*li)->GetWorldAngularVel();
        EXPECT_NEAR(linVel.x, 0, TOL);
        EXPECT_NEAR(linVel.y, 0, TOL);
        EXPECT_NEAR(linVel.z, 0, TOL);
        EXPECT_NEAR(angVel.x, 0, TOL);
        EXPECT_NEAR(angVel.y, 0, TOL);
        EXPECT_NEAR(angVel.z, 0, TOL);
      }
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
