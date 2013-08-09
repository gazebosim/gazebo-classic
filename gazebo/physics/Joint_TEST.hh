/*
 * Copyright 2013 Open Source Robotics Foundation
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

#ifndef _JOINT_TEST_HH_
#define _JOINT_TEST_HH_

#include <string>
#include <sstream>

#include "test/ServerFixture.hh"

#include "gazebo/common/Time.hh"
#include "gazebo/physics/physics.hh"

using namespace gazebo;

class Joint_TEST : public ServerFixture
{
  protected: Joint_TEST() : ServerFixture(), spawnCount(0)
             {
             }

  /// \brief Spawn a model with a joint.
  public: physics::JointPtr SpawnJoint(const std::string &_type,
                                       bool _wait = true)
          {
            msgs::Factory msg;
            std::ostringstream modelStr;
            std::ostringstream modelName;
            modelName << "joint_model" << this->spawnCount++;

            modelStr << "<sdf version='" << SDF_VERSION << "'>"
              << "<model name ='" << modelName.str() << "'>"
              << "  <link name='parent'>"
              << "  </link>"
              << "  <link name='child'>"
              << "  </link>"
              << "  <joint name='joint' type='" << _type << "'>"
              << "    <pose>0 0 0  0 0 0</pose>"
              << "    <parent>parent</parent>"
              << "    <child>child</child>"
              << "    <axis>"
              << "      <xyz>0 0 1</xyz>"
              << "    </axis>"
              << "  </joint>"
              << "</model>";

            msg.set_sdf(modelStr.str());
            this->factoryPub->Publish(msg);

            while (_wait && !this->HasEntity(modelName.str()))
              common::Time::MSleep(100);

            physics::JointPtr joint;
            if (_wait)
            {
              physics::WorldPtr world = physics::get_world("default");
              EXPECT_TRUE(world != NULL);
              physics::ModelPtr model = world->GetModel(modelName.str());
              EXPECT_TRUE(model!= NULL);
              joint = model->GetJoint("joint");
            }
            return joint;
          }

  /// \brief Count of spawned models, used to ensure unique model names.
  private: unsigned int spawnCount;
};

#endif
