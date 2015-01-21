/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#include "test/ServerFixture.hh"
#include "gazebo/physics/Presets.hh"

using namespace gazebo;

class PresetsTest : public ServerFixture,
                          public testing::WithParamInterface<const char*>
{
}

void TestODE()
{

  {
    // Test ODE profile
    // got to load with physics engine
    Load("test/worlds/presets.world", false, "ode");

    physics::WorldPtr world = physics::get_world("default");
    physics::ODEPhysicsPtr physicsEngine = dynamic_cast<physics::ODEPhysicsPtr>(world->GetPhysicsEngine());

    // TODO: Set to fast
    EXPECT_FLOAT_EQ(physicsEngine->GetMaxStepSize(), 0.01);
    EXPECT_EQ(physicsEngine->GetMaxIterations(), 50);
    

    // Set to balanced

    // Set to accurate
  }
}

void TestSimbody()
{
  {
    Load("test/worlds/presets.world", false, "simbody");

    physics::WorldPtr world = physics::get_world("default");

    // TODO: Set to fast
    EXPECT_FLOAT_EQ(physicsEngine->GetMaxStepSize(), 0.01);

    // Set to balanced

    // Set to accurate
  }
}
