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

typedef std::tr1::tuple<std::string, std::string> std_string2;

class Joint_TEST : public ServerFixture,
                   public ::testing::WithParamInterface<std_string2>
{
  protected: Joint_TEST() : ServerFixture(), spawnCount(0)
             {
             }

  public: virtual void SetUp()
          {
            const ::testing::TestInfo* const test_info =
              ::testing::UnitTest::GetInstance()->current_test_info();
            if (test_info->value_param())
            {
              gzdbg << "Params: " << test_info->value_param() << std::endl;
              std::tr1::tie(this->physicsEngine, this->jointType) = GetParam();
            }
          }

  /// \brief Spawn model with each type of joint.
  /// \param[in] _physicsEngine Type of physics engine to use.
  /// \param[in] _jointType Type of joint to spawn and test.
  public: void SpawnJointTypes(const std::string &_physicsEngine,
                               const std::string &_jointType);

  /// \brief Spawn a model with a joint connecting to the world. The function
  ///        will wait for duration _wait for the model to spawn and attempt
  ///        to return a pointer to the spawned joint. This function is not
  ///        guaranteed to return a valid JointPtr, so the output should be
  ///        checked.
  /// \param[in] _type Type of joint to create.
  /// \param[in] _worldChild Flag to set child link to the world.
  /// \param[in] _worldParent Flag to set parent link to the world.
  /// \param[in] _wait Length of time to wait for model to spawn in order
  ///                  to return Joint pointer.
  public: physics::JointPtr SpawnJoint(const std::string &_type,
                                       bool _worldChild = false,
                                       bool _worldParent = false,
                                       common::Time _wait = common::Time(99, 0))
          {
            msgs::Factory msg;
            std::ostringstream modelStr;
            std::ostringstream modelName;
            modelName << "joint_model" << this->spawnCount++;

            modelStr
              << "<sdf version='" << SDF_VERSION << "'>"
              << "<model name ='" << modelName.str() << "'>";
            if (!_worldParent)
            {
              modelStr
                << "  <link name='parent'>"
                << "  </link>";
            }
            if (!_worldChild)
            {
              modelStr
                << "  <link name='child'>"
                << "  </link>";
            }
            modelStr
              << "  <joint name='joint' type='" << _type << "'>"
              << "    <pose>0 0 0  0 0 0</pose>";
            if (_worldParent)
              modelStr << "    <parent>world</parent>";
            else
              modelStr << "    <parent>parent</parent>";
            if (_worldChild)
              modelStr << "    <child>world</child>";
            else
              modelStr << "    <child>child</child>";
            modelStr
              << "    <axis>"
              << "      <xyz>0 0 1</xyz>"
              << "    </axis>";
            modelStr
              << "  </joint>"
              << "</model>";

            msg.set_sdf(modelStr.str());
            this->factoryPub->Publish(msg);

            physics::JointPtr joint;
            if (_wait != common::Time::Zero)
            {
              common::Time wallStart = common::Time::GetWallTime();
              unsigned int waitCount = 0;
              while (_wait > (common::Time::GetWallTime() - wallStart) &&
                     !this->HasEntity(modelName.str()))
              {
                common::Time::MSleep(100);
                if (++waitCount % 10 == 0)
                {
                  gzwarn << "Waiting " << waitCount / 10 << " seconds for "
                         << _type << " joint to spawn." << std::endl;
                }
              }
              if (this->HasEntity(modelName.str()) && waitCount >= 10)
                gzwarn << _type << " joint has spawned." << std::endl;

              physics::WorldPtr world = physics::get_world("default");
              if (world != NULL)
              {
                physics::ModelPtr model = world->GetModel(modelName.str());
                if (model != NULL)
                  joint = model->GetJoint("joint");
              }
            }
            return joint;
          }

  /// \brief Physics engine for test.
  protected: std::string physicsEngine;

  /// \brief Joint type for test.
  protected: std::string jointType;

  /// \brief Count of spawned models, used to ensure unique model names.
  private: unsigned int spawnCount;
};

#endif
