/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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

#ifndef GAZEBO_TEST_INTEGRATION_JOINT_TEST_HH_
#define GAZEBO_TEST_INTEGRATION_JOINT_TEST_HH_

#include <string>
#include <sstream>
#include <ignition/math/Inertial.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/SemanticVersion.hh>
#include <ignition/math/Vector3.hh>
#include <sdf/sdf_config.h>

#include "gazebo/test/ServerFixture.hh"

#include "gazebo/common/Time.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"

using namespace gazebo;

typedef std::tuple<const char *, const char *> std_string2;

class JointTest : public ServerFixture,
                  public ::testing::WithParamInterface<std_string2>
{
  protected: JointTest() : ServerFixture()
             {
             }

  /// \brief Test Joint::GetInertiaRatio.
  /// \param[in] _physicsEngine Type of physics engine to use.
  public: void GetInertiaRatio(const std::string &_physicsEngine);

  /// \brief Test spring dampers
  /// \param[in] _physicsEngine Type of physics engine to use.
  public: void SpringDamperTest(const std::string &_physicsEngine);

  /// \brief Create and destroy joints repeatedly, monitors memory usage.
  /// \param[in] _physicsEngine Type of physics engine to use.
  public: void JointCreationDestructionTest(const std::string &_physicsEngine);

  /// \brief Create joints dynamically and verify that they will be visualized.
  /// \param[in] _physicsEngine Type of physics engine to use.
  public: void DynamicJointVisualization(const std::string &_physicsEngine);

  // Documentation inherited.
  public: virtual void SetUp()
          {
            const ::testing::TestInfo *const test_info =
              ::testing::UnitTest::GetInstance()->current_test_info();
            if (test_info->value_param())
            {
              gzdbg << "Params: " << test_info->value_param() << std::endl;
              this->physicsEngine = std::get<0>(GetParam());
              this->jointType = std::get<1>(GetParam());
            }
          }

  /// \brief Class to hold parameters for spawning joints.
  public: class SpawnJointOptions
  {
    /// \brief Constructor.
    public: SpawnJointOptions()
            {
            }

    /// \brief Destructor.
    public: ~SpawnJointOptions()
            {
            }

    /// \brief Type of joint to create.
    public: std::string type;

    /// \brief Flag to set child link to the world.
    public: bool worldChild = false;

    /// \brief Flag to set parent link to the world.
    public: bool worldParent = false;

    /// \brief Length of time to wait for model to spawn in order to return
    ///        Joint pointer.
    public: common::Time wait = common::Time(99, 0);

    /// \brief Model pose for spawned model.
    public: ignition::math::Pose3d modelPose;

    /// \brief Child link pose for spawned model.
    public: ignition::math::Pose3d childLinkPose;

    /// \brief Parent link pose for spawned model.
    public: ignition::math::Pose3d parentLinkPose;

    /// \brief Flag to disable including link pose per issue #978.
    public: bool noLinkPose = false;

    /// \brief Joint pose for spawned joint.
    public: ignition::math::Pose3d jointPose;

    /// \brief Axis value for spawned joint.
    public: ignition::math::Vector3d axis = ignition::math::Vector3d::UnitX;

    /// \brief Use parent model frame (#494)
    public: bool useParentModelFrame = false;

    /// \brief Use child link inertia.
    public: bool useChildLinkInertia = false;

    /// \brief Child link inertia.
    public: ignition::math::Inertiald childLinkInertia;
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
  public: physics::JointPtr SpawnJoint(const SpawnJointOptions &_opt,
      const std::string &_sdfVersion = std::string(SDF_VERSION))          {
            msgs::Factory msg;
            std::ostringstream modelStr;
            std::string modelName = this->GetUniqueString("joint_model");
            ignition::math::SemanticVersion version(_sdfVersion);

            modelStr
              << "<sdf version='" << _sdfVersion << "'>"
              << "<model name ='" << modelName << "'>"
              << "  <pose>" << _opt.modelPose << "</pose>";
            if (!_opt.worldParent)
            {
              msgs::Link link;
              link.set_name("parent");
              if (!_opt.noLinkPose)
              {
                msgs::Set(link.mutable_pose(), _opt.parentLinkPose);
              }
              modelStr << msgs::LinkToSDF(link)->ToString("");
            }
            if (!_opt.worldChild)
            {
              msgs::Link link;
              link.set_name("child");
              if (!_opt.noLinkPose)
              {
                msgs::Set(link.mutable_pose(), _opt.childLinkPose);
              }
              if (_opt.useChildLinkInertia)
              {
                msgs::Set(link.mutable_inertial(), _opt.childLinkInertia);
              }
              modelStr << msgs::LinkToSDF(link)->ToString("");
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
            if (version >= ignition::math::SemanticVersion(1, 7))
            {
              modelStr
                << "    <axis>"
                << "      <xyz";
              if (_opt.useParentModelFrame)
              {
                modelStr << " expressed_in='__model__'";
              }
              modelStr
                << ">" << _opt.axis << "</xyz>"
                << "    </axis>";
              // Hack: hardcode a second axis for universal joints
              if (_opt.type == "universal" || _opt.type == "revolute2")
              {
                modelStr
                  << "  <axis2>"
                  << "    <xyz";
                if (_opt.useParentModelFrame)
                {
                  modelStr << " expressed_in='__model__'";
                }
                modelStr
                  << ">" << ignition::math::Vector3d::UnitY << "</xyz>"
                  << "  </axis2>";
              }
            }
            else
            {
              modelStr
                << "    <axis>"
                << "      <xyz>" << _opt.axis << "</xyz>"
                << "      <use_parent_model_frame>" << _opt.useParentModelFrame
                << "      </use_parent_model_frame>"
                << "    </axis>";
              // Hack: hardcode a second axis for universal joints
              if (_opt.type == "universal" || _opt.type == "revolute2")
              {
                modelStr
                  << "  <axis2>"
                  << "    <xyz>" << ignition::math::Vector3d::UnitY << "</xyz>"
                  << "    <use_parent_model_frame>" << _opt.useParentModelFrame
                  << "    </use_parent_model_frame>"
                  << "  </axis2>";
              }
            }
            modelStr
              << "  </joint>"
              << "</model>"
              << "</sdf>";

            msg.set_sdf(modelStr.str());
            this->factoryPub->Publish(msg);

            physics::JointPtr joint;
            if (_opt.wait != common::Time::Zero)
            {
              common::Time wallStart = common::Time::GetWallTime();
              unsigned int waitCount = 0;
              while (_opt.wait > (common::Time::GetWallTime() - wallStart) &&
                     !this->HasEntity(modelName))
              {
                common::Time::MSleep(100);
                if (++waitCount % 10 == 0)
                {
                  gzwarn << "Waiting " << waitCount / 10 << " seconds for "
                         << _opt.type << " joint to spawn." << std::endl;
                }
              }
              if (this->HasEntity(modelName) && waitCount >= 10)
                gzwarn << _opt.type << " joint has spawned." << std::endl;

              physics::WorldPtr world = physics::get_world("default");
              if (world != NULL)
              {
                physics::ModelPtr model = world->ModelByName(modelName);
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
};
#endif
