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

  /// \brief Spawn model with each type of joint.
  /// \param[in] _physicsEngine Type of physics engine to use.
  public: void SpawnJointTypes(const std::string &_physicsEngine);

  /// \brief Spawn model with rotational joints. Set velocity on parent
  ///        and make sure child follows. Also attach to world and make
  ///        sure it doesn't fall.
  /// \param[in] _physicsEngine Type of physics engine to use.
  public: void SpawnJointRotational(const std::string &_physicsEngine);

  /// \brief Class to hold parameters for spawning joints.
  public: class SpawnJointOptions
  {
    /// \brief Constructor.
    public: SpawnJointOptions() : worldChild(false), worldParent(false),
              axis(math::Vector3(0, 0, 1))
            {
            }

    /// \brief Destructor.
    public: ~SpawnJointOptions()
            {
            }

    /// \brief Type of joint to create.
    public: std::string type;

    /// \brief Flag to set child link to the world.
    public: bool worldChild;

    /// \brief Flag to set parent link to the world.
    public: bool worldParent;

    /// \brief Length of time to wait for model to spawn in order to return
    ///        Joint pointer.
    public: common::Time wait;

    /// \brief Model pose for spawned model.
    public: math::Pose modelPose;

    /// \brief Child link pose for spawned model.
    public: math::Pose childLinkPose;

    /// \brief Parent link pose for spawned model.
    public: math::Pose parentLinkPose;

    /// \brief Joint pose for spawned joint.
    public: math::Pose jointPose;

    /// \brief Axis value for spawned joint.
    public: math::Vector3 axis;
  };

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
            SpawnJointOptions opt;
            opt.type = _type;
            opt.worldChild = _worldChild;
            opt.worldParent = _worldParent;
            opt.wait = _wait;
            return SpawnJoint(opt);
          }

  /// \brief Spawn a model with a joint connecting to the world. The function
  ///        will wait for duration _wait for the model to spawn and attempt
  ///        to return a pointer to the spawned joint. This function is not
  ///        guaranteed to return a valid JointPtr, so the output should be
  ///        checked.
  /// \param[in] _opt Options for spawned model and joint.
  public: physics::JointPtr SpawnJoint(const SpawnJointOptions _opt)
          {
            msgs::Factory msg;
            std::ostringstream modelStr;
            std::ostringstream modelName;
            modelName << "joint_model" << this->spawnCount++;

            modelStr
              << "<sdf version='" << SDF_VERSION << "'>"
              << "<model name ='" << modelName.str() << "'>"
              << "  <pose>" << _opt.modelPose << "</pose>";
            if (!_opt.worldParent)
            {
              modelStr
                << "  <link name='parent'>"
                << "    <pose>" << _opt.parentLinkPose << "</pose>"
                << "  </link>";
            }
            if (!_opt.worldChild)
            {
              modelStr
                << "  <link name='child'>"
                << "    <pose>" << _opt.childLinkPose << "</pose>"
                << "  </link>";
            }
            modelStr
              << "  <joint name='joint' type='" << _opt.type << "'>"
              << "    <pose>" << _opt.jointPose << "</pose>";
            if (_opt.worldParent)
              modelStr << "    <parent>world</parent>";
            else
              modelStr << "    <parent>parent</parent>";
            if (_opt.worldChild)
              modelStr << "    <child>world</child>";
            else
              modelStr << "    <child>child</child>";
            modelStr
              << "    <axis>"
              << "      <xyz>" << _opt.axis << "</xyz>"
              << "    </axis>";
            modelStr
              << "  </joint>"
              << "</model>";

            msg.set_sdf(modelStr.str());
            this->factoryPub->Publish(msg);

            physics::JointPtr joint;
            if (_opt.wait != common::Time::Zero)
            {
              common::Time wallStart = common::Time::GetWallTime();
              unsigned int waitCount = 0;
              while (_opt.wait > (common::Time::GetWallTime() - wallStart) &&
                     !this->HasEntity(modelName.str()))
              {
                common::Time::MSleep(100);
                if (++waitCount % 10 == 0)
                {
                  gzwarn << "Waiting " << waitCount / 10 << " seconds for "
                         << _opt.type << " joint to spawn." << std::endl;
                }
              }
              if (this->HasEntity(modelName.str()) && waitCount >= 10)
                gzwarn << _opt.type << " joint has spawned." << std::endl;

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

  /// \brief Count of spawned models, used to ensure unique model names.
  private: unsigned int spawnCount;
};

#endif
