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

#include <boost/bind.hpp>
#include <functional>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include "VariableGearboxPlugin.hh"

namespace gazebo
{
class VariableGearboxPluginPrivate
{
  /// \brief Parent model pointer.
  public: physics::ModelPtr model;

  /// \brief Gearbox joint.
  public: physics::JointPtr gearbox;

  /// \brief Input joint.
  public: physics::JointPtr inputJoint;

  /// \brief Output joint.
  public: physics::JointPtr outputJoint;

  /// \brief World update connection.
  public: event::ConnectionPtr updateConnection;
};

/////////////////////////////////////////////////
VariableGearboxPlugin::VariableGearboxPlugin()
  : dataPtr(new VariableGearboxPluginPrivate)
{
}

/////////////////////////////////////////////////
VariableGearboxPlugin::~VariableGearboxPlugin()
{
}

/////////////////////////////////////////////////
void VariableGearboxPlugin::Load(physics::ModelPtr _parent,
                                 sdf::ElementPtr /*_sdf*/)
{
  this->dataPtr->model = _parent;

  const std::string jointName = "gearbox_demo";
  auto joint = this->dataPtr->model->GetJoint(jointName);
  if (joint == nullptr || !joint->HasType(physics::Base::GEARBOX_JOINT))
  {
    gzerr << "Could not find a joint named " << jointName << std::endl;
    return;
  }
  this->dataPtr->gearbox = joint;

  auto parentLink = joint->GetParent();
  if (parentLink == nullptr)
  {
    gzerr << "Could not find parent link." << std::endl;
    return;
  }
  std::cerr << "Checking " << parentLink->GetScopedName()
            << " for its joints."
            << std::endl;

  {
    auto joints = parentLink->GetParentJoints();
    if (joints.size() != 1)
    {
      gzerr << "link [" << parentLink->GetScopedName()
            << "] is child of more than 1 joint, not sure which one to pick."
            << std::endl;
      return;
    }
    this->dataPtr->inputJoint = joints.front();
  }
  std::cerr << "Using " << this->dataPtr->inputJoint->GetScopedName()
            << " as input joint."
            << std::endl;

  auto childLink = joint->GetChild();
  if (childLink == nullptr)
  {
    gzerr << "Could not find child link." << std::endl;
    return;
  }
  std::cerr << "Checking " << childLink->GetScopedName()
            << " for its joints."
            << std::endl;

  {
    auto joints = childLink->GetParentJoints();
    if (joints.size() < 1 || joints.size() > 2)
    {
      gzerr << "link [" << childLink->GetScopedName()
            << "] is child of not 1 or 2 joints, not sure which one to pick."
            << std::endl;
      return;
    }
    for (auto j : joints)
    {
      gzerr << " joint " << j->GetScopedName()
            << ", gearbox?"
            << j->HasType(physics::Base::GEARBOX_JOINT)
            << std::endl;
    }
    this->dataPtr->outputJoint = joints.front();
  }
  std::cerr << "Using " << this->dataPtr->outputJoint->GetScopedName()
            << " as output joint."
            << std::endl;

  this->dataPtr->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&VariableGearboxPlugin::OnUpdate, this, _1));
}

/////////////////////////////////////////////////
void VariableGearboxPlugin::OnUpdate(const common::UpdateInfo & /*_info*/)
{
  const double input0 = 1.2;
  const double output0 = -1.2;
  const double ratio1 = -1;
  const double ratio2 = -20;
  const double dinput = 0.6;
  const double dratio = ratio2 - ratio1;
  double ratio;
  double refInput;
  double refOutput;
  double inputAngle = this->dataPtr->inputJoint->GetAngle(0).Radian();
  // double outputAngle = this->dataPtr->outputJoint->GetAngle(0).Radian();
  if (inputAngle < input0)
  {
    ratio = ratio1;
    refInput = input0;
    refOutput = output0;
  }
  else if (inputAngle < input0+dinput)
  {
    ratio = ratio1 + dratio * (inputAngle - input0) / dinput;
    refInput = inputAngle;
    refOutput = output0 + ratio1 * (inputAngle - input0)
              + 0.5*dratio / dinput * std::pow(inputAngle - input0, 2);
  }
  else
  {
    ratio = ratio2;
    refInput = input0 + dinput;
    refOutput = output0 + 0.5 * (ratio1 + ratio2) * dinput;
  }
  ratio = -ratio;

  this->dataPtr->gearbox->SetParam("reference_angle1", 0, refOutput);
  this->dataPtr->gearbox->SetParam("reference_angle2", 0, refInput);
  this->dataPtr->gearbox->SetParam("ratio", 0, ratio);
  // std::cerr << "input " << inputAngle
  //           << " , " << refInput << " ref"
  //           << " : " << ratio
  //           << " : ref " << refOutput
  //           << " , " << outputAngle << " output"
  //           << std::endl;
}

GZ_REGISTER_MODEL_PLUGIN(VariableGearboxPlugin)
}
