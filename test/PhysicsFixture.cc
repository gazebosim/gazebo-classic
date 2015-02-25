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


#include "test/PhysicsFixture.hh"
#include "gazebo/physics/physics.hh"

using namespace gazebo;

/////////////////////////////////////////////////
PhysicsFixture::PhysicsFixture() : ServerFixture()
{
}

/////////////////////////////////////////////////
void PhysicsFixture::LoadWorld(const std::string &_worldFilename,
                          bool _paused,
                          const std::string &_physicsEngine)
{
  ServerFixture::Load(_worldFilename, _paused, _physicsEngine);
  this->world = physics::get_world();
  ASSERT_TRUE(this->world != NULL);

  this->physics = world->GetPhysicsEngine();
  ASSERT_TRUE(this->physics != NULL);
  EXPECT_EQ(this->physics->GetType(), _physicsEngine);
}

/////////////////////////////////////////////////
physics::ModelPtr PhysicsFixture::GetModel(const std::string &_modelName)
{
  EXPECT_TRUE(this->world != NULL);
  physics::ModelPtr model = this->world->GetModel(_modelName);
  EXPECT_TRUE(model != NULL);
  return model;
}
