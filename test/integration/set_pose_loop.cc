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
  double pub_rate;
  double last_update_time;
  double elapsed_wall_time;


  physics::JointPtr joint_01 = model->GetJoint("model_1::joint_01");
  physics::JointPtr joint_12 = model->GetJoint("model_1::joint_12");
  physics::JointPtr joint_23 = model->GetJoint("model_1::joint_23");
  physics::JointPtr joint_34 = model->GetJoint("model_1::joint_34");
  physics::JointPtr joint_45 = model->GetJoint("model_1::joint_45");
  physics::JointPtr joint_52 = model->GetJoint("model_1::joint_52");

  start_time = world->GetSimTime().Double();
  start_wall_time = world->GetRealTime().Double();
  test_duration = 10;
  pub_rate = 10.0;
  gzdbg << " -------------------------------------------------------------\n";
  gzdbg << " Publishing Joint::SetAngle at ["
    << pub_rate << "] Hz.\n";
  last_update_time = start_time;
  while (world->GetSimTime().Double() < start_time + test_duration)
    if (world->GetSimTime().Double() - last_update_time >= (1.0/pub_rate))
    {
      last_update_time = world->GetSimTime().Double();
      // gzdbg << "setting link poses without violation\n";
      // double cur_time = world->GetSimTime().Double();
      joint_01->SetAngle(0, 0.1);
      joint_12->SetAngle(0, 0.1);
      joint_23->SetAngle(0, 0.1);
      joint_34->SetAngle(0, 0.1);
      joint_45->SetAngle(0, 0.1);
      joint_52->SetAngle(0, 0.1);

      joint_01->SetAngle(0, 0.2);
      joint_12->SetAngle(0, 0.2);
      joint_23->SetAngle(0, 0.2);
      joint_34->SetAngle(0, 0.2);
      joint_45->SetAngle(0, 0.2);
      joint_52->SetAngle(0, 0.2);
    }
  elapsed_wall_time = world->GetRealTime().Double() - start_wall_time;
  gzdbg << "  elapsed sim time [" << test_duration
    << "] elapsed wall time [" << elapsed_wall_time
    << "] sim performance [" << test_duration / elapsed_wall_time
    << "]\n";




  world->EnablePhysicsEngine(false);

  start_time = world->GetSimTime().Double();
  start_wall_time = world->GetRealTime().Double();
  test_duration = 20;
  pub_rate = 10.0;
  gzdbg << " -------------------------------------------------------------\n";
  gzdbg << " Publishing Joint::SetAngle at ["
    << pub_rate << "] Hz with real time duration.\n";
  last_update_time = start_wall_time;
  while (world->GetRealTime().Double() < start_wall_time + test_duration)
    if (world->GetRealTime().Double() - last_update_time >= (1.0/pub_rate))
    {
      last_update_time = world->GetRealTime().Double();
      joint_01->SetAngle(0,
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX));
      joint_12->SetAngle(0,
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX));
      joint_23->SetAngle(0,
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX));
      joint_34->SetAngle(0,
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX));
      joint_45->SetAngle(0,
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX));
      joint_52->SetAngle(0,
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX));
      sleep(1);
    }
  test_duration = world->GetSimTime().Double() - start_time;
  elapsed_wall_time = world->GetRealTime().Double() - start_wall_time;
  gzdbg << "  elapsed sim time [" << test_duration
    << "] elapsed wall time [" << elapsed_wall_time
    << "] sim performance [" << test_duration / elapsed_wall_time
    << "]\n";


  world->EnablePhysicsEngine(true);
















  physics::LinkPtr link_1 = model->GetLink("link_1");
  physics::LinkPtr link_2 = model->GetLink("link_2");
  physics::LinkPtr link_3 = model->GetLink("link_3");
  physics::LinkPtr link_4 = model->GetLink("link_4");
  physics::LinkPtr link_5 = model->GetLink("link_5");

  EXPECT_TRUE(link_1 != NULL);

  start_time = world->GetSimTime().Double();
  start_wall_time = world->GetRealTime().Double();
  test_duration = 10;
  pub_rate = 2.0;
  gzdbg << " -------------------------------------------------------------\n";
  gzdbg << " Publishing SetWorld Pose at ["
    << pub_rate << "] Hz without constraint violation.\n";
  last_update_time = start_time;
  while (world->GetSimTime().Double() < start_time + test_duration)
    if (world->GetSimTime().Double() - last_update_time >= (1.0/pub_rate))
    {
      // gzdbg << "setting link poses without violation\n";
      // double cur_time = world->GetSimTime().Double();
      last_update_time = world->GetSimTime().Double();
      link_1->SetWorldPose(math::Pose(0,    0, 3, 0,        0, 0));
      link_2->SetWorldPose(math::Pose(1.00, 0, 3, 0,        0, 0));
      link_3->SetWorldPose(math::Pose(2.00, 0, 3, 0, 0.5*M_PI, 0));
      link_4->SetWorldPose(math::Pose(2.00, 1, 3, 0, 1.0*M_PI, 0));
      link_5->SetWorldPose(math::Pose(1.00, 1, 3, 0, 1.5*M_PI, 0));
    }
  elapsed_wall_time = world->GetRealTime().Double() - start_wall_time;
  gzdbg << "  elapsed sim time [" << test_duration
    << "] elapsed wall time [" << elapsed_wall_time
    << "] sim performance [" << test_duration / elapsed_wall_time
    << "]\n";

  start_time = world->GetSimTime().Double();
  start_wall_time = world->GetRealTime().Double();
  test_duration = 10;
  pub_rate = 1.0;
  gzdbg << " -------------------------------------------------------------\n";
  gzdbg << " Publishing SetWorld Pose at ["
    << pub_rate << "] Hz without constraint violation.\n";
  last_update_time = start_time;
  while (world->GetSimTime().Double() < start_time + test_duration)
    if (world->GetSimTime().Double() - last_update_time >= (1.0/pub_rate))
    {
      last_update_time = world->GetSimTime().Double();
      // gzdbg << "setting link poses without violation\n";
      // double cur_time = world->GetSimTime().Double();
      link_1->SetWorldPose(math::Pose(0,    0, 3, 0, 0, 0));
      link_2->SetWorldPose(math::Pose(1.00, 0, 3, 0, 0, 0));
      link_3->SetWorldPose(math::Pose(2.00, 0, 3, 0, 0.5*M_PI, 0));
      link_4->SetWorldPose(math::Pose(2.00, 1, 3, 0, 1.0*M_PI, 0));
      link_5->SetWorldPose(math::Pose(1.00, 1, 3, 0, 1.5*M_PI, 0));
    }
  elapsed_wall_time = world->GetRealTime().Double() - start_wall_time;
  gzdbg << "  elapsed sim time [" << test_duration
    << "] elapsed wall time [" << elapsed_wall_time
    << "] sim performance [" << test_duration / elapsed_wall_time
    << "]\n";




  // set random pose within joint limit
  start_time = world->GetSimTime().Double();
  start_wall_time = world->GetRealTime().Double();
  test_duration = 20;
  pub_rate = 2.0;
  gzdbg << " -------------------------------------------------------------\n";
  gzdbg << " Publishing SetWorld Pose at ["
    << pub_rate << "] Hz with less constraint violation.\n";
  last_update_time = start_time;
  while (world->GetSimTime().Double() < start_time + test_duration)
    // if (world->GetSimTime().Double() - last_update_time >= (1.0/pub_rate))
  {
    last_update_time = world->GetSimTime().Double();
    math::Pose p;
    p = math::Pose(
        static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
        static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
        static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX) + 3.0,
        static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
        static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
        static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX));
    link_1->SetWorldPose(p);
    p = math::Pose(
        static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX) + 1.0,
        static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
        static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX) + 3.0,
        static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
        static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
        static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX));
    link_2->SetWorldPose(p);
    p = math::Pose(
        static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX) + 2.0,
        static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
        static_cast<double>(rand_r(&seed))/
        static_cast<double>(RAND_MAX) + 3.0,
        static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
        static_cast<double>(rand_r(&seed))/
        static_cast<double>(RAND_MAX) + 0.5*M_PI,
        static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX));
    link_3->SetWorldPose(p);
    p = math::Pose(
        static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX) + 2.0,
        static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX) + 1.0,
        static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX) + 3.0,
        static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
        static_cast<double>(rand_r(&seed))/
        static_cast<double>(RAND_MAX) + M_PI,
        static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX));
    link_4->SetWorldPose(p);
    p = math::Pose(
        static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX) + 1.0,
        static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX) + 1.0,
        static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX) + 3.0,
        static_cast<double>(rand_r(&seed))/
        static_cast<double>(RAND_MAX) + 1.5*M_PI,
        static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
        static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX));
    link_5->SetWorldPose(p);

    math::Vector3 v;
    v = math::Vector3(
        static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
        static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
        static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX));
    link_1->SetAngularVel(v);
    v = math::Vector3(
        static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
        static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
        static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX));
    link_1->SetLinearVel(v);
    v = math::Vector3(
        static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
        static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
        static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX));
    link_2->SetAngularVel(v);
    v = math::Vector3(
        static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
        static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
        static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX));
    link_2->SetLinearVel(v);
    v = math::Vector3(
        static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
        static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
        static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX));
    link_3->SetAngularVel(v);
    v = math::Vector3(
        static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
        static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
        static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX));
    link_3->SetLinearVel(v);
    v = math::Vector3(
        static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
        static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
        static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX));
    link_4->SetAngularVel(v);
    v = math::Vector3(
        static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
        static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
        static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX));
    link_4->SetLinearVel(v);
    v = math::Vector3(
        static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
        static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
        static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX));
    link_5->SetAngularVel(v);
    v = math::Vector3(
        static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
        static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
        static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX));
    link_5->SetLinearVel(v);
  }
  elapsed_wall_time = world->GetRealTime().Double() - start_wall_time;
  gzdbg << "  elapsed sim time [" << test_duration
    << "] elapsed wall time [" << elapsed_wall_time
    << "] sim performance [" << test_duration / elapsed_wall_time
    << "]\n";



  // set random pose outside of joint limit
  start_time = world->GetSimTime().Double();
  start_wall_time = world->GetRealTime().Double();
  test_duration = 20;
  pub_rate = 1000.0;
  gzdbg << " -------------------------------------------------------------\n";
  gzdbg << " Publishing SetWorld Pose at ["
    << pub_rate << "] Hz with more random constraint violation.\n";
  last_update_time = start_time;
  while (world->GetSimTime().Double() < start_time + test_duration)
    if (world->GetSimTime().Double() - last_update_time >= (1.0/pub_rate))
    {
      last_update_time = world->GetSimTime().Double();
      math::Pose p;
      p = math::Pose(
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(&seed))/
          static_cast<double>(RAND_MAX) + 3.0,
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(&seed))/
          static_cast<double>(RAND_MAX) + 1.57079,
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX));
      link_1->SetWorldPose(p);
      p = math::Pose(
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(&seed))/
          static_cast<double>(RAND_MAX) + 2.0,
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(&seed))/
          static_cast<double>(RAND_MAX) + 1.57079,
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX));
      link_2->SetWorldPose(p);
      p = math::Pose(
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(&seed))/
          static_cast<double>(RAND_MAX) + 1.0,
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(&seed))/
          static_cast<double>(RAND_MAX) + 1.57079,
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX));
      link_3->SetWorldPose(p);
      p = math::Pose(
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(&seed))/
          static_cast<double>(RAND_MAX) + 1.0,
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(&seed))/
          static_cast<double>(RAND_MAX) + 1.57079,
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX));
      link_4->SetWorldPose(p);
      p = math::Pose(
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(&seed))/
          static_cast<double>(RAND_MAX) + 1.0,
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(&seed))/
          static_cast<double>(RAND_MAX) + 1.57079,
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX));
      link_5->SetWorldPose(p);

      math::Vector3 v;
      v = math::Vector3(
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX));
      link_1->SetAngularVel(v);
      v = math::Vector3(
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX));
      link_1->SetLinearVel(v);
      v = math::Vector3(
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX));
      link_2->SetAngularVel(v);
      v = math::Vector3(
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX));
      link_2->SetLinearVel(v);
      v = math::Vector3(
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX));
      link_3->SetAngularVel(v);
      v = math::Vector3(
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX));
      link_3->SetLinearVel(v);
      v = math::Vector3(
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX));
      link_4->SetAngularVel(v);
      v = math::Vector3(
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX));
      link_4->SetLinearVel(v);
      v = math::Vector3(
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX));
      link_5->SetAngularVel(v);
      v = math::Vector3(
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX));
      link_5->SetLinearVel(v);
    }
  elapsed_wall_time = world->GetRealTime().Double() - start_wall_time;
  gzdbg << "  elapsed sim time [" << test_duration
    << "] elapsed wall time [" << elapsed_wall_time
    << "] sim performance [" << test_duration / elapsed_wall_time
    << "]\n";



  start_time = world->GetSimTime().Double();
  start_wall_time = world->GetRealTime().Double();
  test_duration = 10;
  pub_rate = 1000.0;
  gzdbg << " -------------------------------------------------------------\n";
  gzdbg << " Publishing Set*Vel at ["
    << pub_rate << "] Hz with random velocities.\n";
  last_update_time = start_time;
  while (world->GetSimTime().Double() < start_time + test_duration)
    if (world->GetSimTime().Double() - last_update_time >= (1.0/pub_rate))
    {
      last_update_time = world->GetSimTime().Double();
      // gzdbg << "setting link poses with violation\n";
      math::Vector3 v;
      v = math::Vector3(
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX));
      link_1->SetAngularVel(v);
      v = math::Vector3(
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX));
      link_1->SetLinearVel(v);
      v = math::Vector3(
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX));
      link_2->SetAngularVel(v);
      v = math::Vector3(
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX));
      link_2->SetLinearVel(v);
      v = math::Vector3(
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX));
      link_3->SetAngularVel(v);
      v = math::Vector3(
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX));
      link_3->SetLinearVel(v);
      v = math::Vector3(
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX));
      link_4->SetAngularVel(v);
      v = math::Vector3(
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX));
      link_4->SetLinearVel(v);
      v = math::Vector3(
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX));
      link_5->SetAngularVel(v);
      v = math::Vector3(
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX),
          static_cast<double>(rand_r(&seed))/static_cast<double>(RAND_MAX));
      link_5->SetLinearVel(v);
    }
  elapsed_wall_time = world->GetRealTime().Double() - start_wall_time;
  gzdbg << "  elapsed sim time [" << test_duration
    << "] elapsed wall time [" << elapsed_wall_time
    << "] sim performance [" << test_duration / elapsed_wall_time
    << "]\n";

  start_time = world->GetSimTime().Double();
  start_wall_time = world->GetRealTime().Double();
  test_duration = 20;
  pub_rate = 500.0;
  gzdbg << " -------------------------------------------------------------\n";
  gzdbg << " Publishing Set*Vel at ["
    << pub_rate << "] Hz with velocity decay.\n";
  last_update_time = start_time;
  while (world->GetSimTime().Double() < start_time + test_duration)
    if (world->GetSimTime().Double() - last_update_time >= (1.0/pub_rate))
    {
      last_update_time = world->GetSimTime().Double();
      // gzdbg << "setting link poses with violation\n";
      link_1->SetAngularVel(link_1->GetWorldAngularVel() * 0.999);
      link_1->SetLinearVel(link_1->GetWorldLinearVel()  * 0.999);
      link_2->SetAngularVel(link_2->GetWorldAngularVel() * 0.999);
      link_2->SetLinearVel(link_2->GetWorldLinearVel()  * 0.999);
      link_3->SetAngularVel(link_3->GetWorldAngularVel() * 0.999);
      link_3->SetLinearVel(link_3->GetWorldLinearVel()  * 0.999);
      link_4->SetAngularVel(link_4->GetWorldAngularVel() * 0.999);
      link_4->SetLinearVel(link_4->GetWorldLinearVel()  * 0.999);
      link_5->SetAngularVel(link_5->GetWorldAngularVel() * 0.999);
      link_5->SetLinearVel(link_5->GetWorldLinearVel()  * 0.999);
    }
  elapsed_wall_time = world->GetRealTime().Double() - start_wall_time;
  gzdbg << "  elapsed sim time [" << test_duration
    << "] elapsed wall time [" << elapsed_wall_time
    << "] sim performance [" << test_duration / elapsed_wall_time
    << "]\n";

  EXPECT_EQ(link_3->GetWorldPose(),
      math::Pose(0.292968, 0.612084, 1.43649, -2.07141, 1.50881, -1.19487));
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
