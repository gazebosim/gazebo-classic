/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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

#include <string.h>
#include "ServerFixture.hh"
#include "test/helpers/box_spawn.hh"
#include "test/helpers/helper_physics_generator.hh"

using namespace gazebo;
class BallisticGyroScopic : public ServerFixture,
                            public testing::WithParamInterface<const char*>
{
  public: void Load(const std::string &_physicsEngine);
};

/////////////////////////////////////////////////
void BallisticGyroScopic::Load(const std::string &_physicsEngine)
{
  ServerFixture::Load("worlds/blank.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world();
  ASSERT_TRUE(world);

  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics);

  math::Vector3 gravity = physics->GetGravity();

  BoxSpawn bs;
  BoxSpawn::BoxSpawnOptions opt;
  opt.size.Set(1, 4, 9);
  std::string boxSdf = bs.BoxSpawnString(opt);

  ServerFixture::SpawnSDF(boxSdf);
  physics::ModelPtr model = world->GetModel(opt.name);
  ASSERT_TRUE(model);

  physics::LinkPtr link = model->GetLink();
  ASSERT_TRUE(link);

  math::Vector3 initialLinearVel(1, 4, 9);
  math::Vector3 initialAngularVel(10, 1, 1);
  link->SetLinearVel(initialLinearVel);
  link->SetAngularVel(initialAngularVel);

  math::Matrix3 inertia = link->GetInertial()->GetMOI();

  double totalTime = 10.0;
  double dt = physics->GetMaxStepSize();
  unsigned int steps = ceil(totalTime / dt);
  std::cout << "t (sec),linear velocity error" << std::endl;
  for (unsigned int i = 0; i < steps; ++i)
  {
    world->StepWorld(1);
    double t = world->GetSimTime().Double();
    math::Vector3 linearVel = link->GetWorldLinearVel();
    math::Vector3 linearVelErr = initialLinearVel + gravity * t - linearVel;
    math::Vector3 angularVel = link->GetWorldAngularVel();
    math::Quaternion ori = link->GetWorldPose().rot;
    math::Matrix4 inertiaWorld;
    inertiaWorld = ori.GetInverse().GetAsMatrix3() * inertia;
    math::Vector3 angularMomentum = inertiaWorld * angularVel;
    std::cout << std::fixed << t << std::scientific
       << ',' << linearVelErr.x
       << ',' << linearVelErr.y
       << ',' << linearVelErr.z
       << ',' << angularMomentum.x
       << ',' << angularMomentum.y
       << ',' << angularMomentum.z
              << std::endl;
  }
}

TEST_P(BallisticGyroScopic, Load)
{
  Load(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, BallisticGyroScopic,
                        PHYSICS_ENGINE_VALUES);

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
