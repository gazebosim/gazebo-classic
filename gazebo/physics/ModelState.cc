/*
 * Copyright 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License")
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

#include "gazebo/common/Exception.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/ModelState.hh"

using namespace gazebo;
using namespace physics;

/////////////////////////////////////////////////
ModelState::ModelState()
: State()
{
}

/////////////////////////////////////////////////
ModelState::ModelState(const ModelPtr _model)
: State(_model->GetName(), _model->GetWorld()->GetRealTime(),
        _model->GetWorld()->GetSimTime())
{
  this->pose = _model->GetWorldPose();

  // Copy all the links
  const Link_V links = _model->GetLinks();
  for (Link_V::const_iterator iter = links.begin(); iter != links.end(); ++iter)
  {
    this->linkStates.push_back(LinkState((*iter)));
  }

  // Copy all the joints
  const Joint_V joints = _model->GetJoints();
  for (Joint_V::const_iterator iter = joints.begin();
       iter != joints.end(); ++iter)
  {
    this->jointStates.push_back(JointState(*iter));
  }
}

/////////////////////////////////////////////////
ModelState::ModelState(const sdf::ElementPtr _sdf)
  : State()
{
  this->Load(_sdf);
}

/////////////////////////////////////////////////
ModelState::~ModelState()
{
}

/////////////////////////////////////////////////
void ModelState::Load(const sdf::ElementPtr _elem)
{
  // Set the name
  this->name = _elem->GetValueString("name");

  // Set the model pose
  if (_elem->HasElement("pose"))
    this->pose = _elem->GetValuePose("pose");
  else
    this->pose.Set(0, 0, 0, 0, 0, 0);

  // Set all the links
  this->linkStates.clear();
  if (_elem->HasElement("link"))
  {
    sdf::ElementPtr childElem = _elem->GetElement("link");

    while (childElem)
    {
      this->linkStates.push_back(LinkState(childElem));
      childElem = childElem->GetNextElement("link");
    }
  }

  // Set all the joints
  this->jointStates.clear();
  if (_elem->HasElement("joint"))
  {
    sdf::ElementPtr childElem = _elem->GetElement("joint");

    while (childElem)
    {
      this->jointStates.push_back(JointState(childElem));
      childElem = childElem->GetNextElement("joint");
    }
  }
}

/////////////////////////////////////////////////
const math::Pose &ModelState::GetPose() const
{
  return this->pose;
}

/////////////////////////////////////////////////
bool ModelState::IsZero() const
{
  bool result = true;

  for (std::vector<LinkState>::const_iterator iter = this->linkStates.begin();
       iter != this->linkStates.end() && result; ++iter)
  {
    result = result && (*iter).IsZero();
  }

  for (std::vector<JointState>::const_iterator iter = this->jointStates.begin();
       iter != this->jointStates.end() && result; ++iter)
  {
    result = result && (*iter).IsZero();
  }

  return result && this->pose == math::Pose::Zero;
}

/////////////////////////////////////////////////
unsigned int ModelState::GetLinkStateCount() const
{
  return this->linkStates.size();
}

/////////////////////////////////////////////////
LinkState ModelState::GetLinkState(unsigned int _index) const
{
  if (_index < this->linkStates.size())
    return this->linkStates[_index];

  gzthrow("Index is out of range");
  return LinkState();
}

/////////////////////////////////////////////////
LinkState ModelState::GetLinkState(const std::string &_linkName) const
{
  // Search for the link name
  for (std::vector<LinkState>::const_iterator iter = this->linkStates.begin();
       iter != this->linkStates.end(); ++iter)
  {
    if ((*iter).GetName() == _linkName)
      return *iter;
  }

  gzthrow("Invalid link name[" + _linkName + "]");
  return LinkState();
}

/////////////////////////////////////////////////
bool ModelState::HasLinkState(const std::string &_linkName) const
{
  // Search for the link name
  for (std::vector<LinkState>::const_iterator iter = this->linkStates.begin();
       iter != this->linkStates.end(); ++iter)
  {
    if ((*iter).GetName() == _linkName)
      return true;
  }

  return false;
}

/////////////////////////////////////////////////
const std::vector<LinkState> &ModelState::GetLinkStates() const
{
  return this->linkStates;
}

/////////////////////////////////////////////////
unsigned int ModelState::GetJointStateCount() const
{
  return this->jointStates.size();
}

/////////////////////////////////////////////////
JointState ModelState::GetJointState(unsigned int _index) const
{
  if (_index < this->jointStates.size())
    return this->jointStates[_index];

  gzthrow("Index is out of range");
  return JointState();
}

/////////////////////////////////////////////////
JointState ModelState::GetJointState(const std::string &_jointName) const
{
  for (std::vector<JointState>::const_iterator iter = this->jointStates.begin();
       iter != this->jointStates.end(); ++iter)
  {
    if ((*iter).GetName() == _jointName)
      return *iter;
  }

  gzthrow("Invalid joint name[" + _jointName + "]");
  return JointState();
}

/////////////////////////////////////////////////
bool ModelState::HasJointState(const std::string &_jointName) const
{
  for (std::vector<JointState>::const_iterator iter = this->jointStates.begin();
       iter != this->jointStates.end(); ++iter)
  {
    if ((*iter).GetName() == _jointName)
      return true;
  }

  return false;
}

/////////////////////////////////////////////////
const std::vector<JointState> &ModelState::GetJointStates() const
{
  return this->jointStates;
}

/////////////////////////////////////////////////
ModelState &ModelState::operator=(const ModelState &_state)
{
  State::operator=(_state);

  // Copy the pose
  this->pose = _state.pose;

  // Clear the link and joint states.
  this->linkStates.clear();
  this->jointStates.clear();

  // Copy the link states.
  for (std::vector<LinkState>::const_iterator iter =
       _state.linkStates.begin(); iter != _state.linkStates.end(); ++iter)
  {
    this->linkStates.push_back(LinkState(*iter));
  }

  // Copy the joint states.
  for (std::vector<JointState>::const_iterator iter =
       _state.jointStates.begin(); iter != _state.jointStates.end(); ++iter)
  {
    this->jointStates.push_back(JointState(*iter));
  }

  return *this;
}

/////////////////////////////////////////////////
ModelState ModelState::operator-(const ModelState &_state) const
{
  ModelState result;

  result.name = this->name;
  result.pose.pos = this->pose.pos - _state.pose.pos;
  result.pose.rot = _state.pose.rot.GetInverse() * this->pose.rot;

  // Insert the link state diffs.
  for (std::vector<LinkState>::const_iterator iter =
       this->linkStates.begin(); iter != this->linkStates.end(); ++iter)
  {
    try
    {
      LinkState state = (*iter) - _state.GetLinkState((*iter).GetName());
      if (!state.IsZero())
        result.linkStates.push_back(state);
    }
    catch(common::Exception &)
    {
      // Ignore exception, which is just the fact that a link state may not
      // have been recorded.
    }
  }

  // Insert the joint state diffs.
  for (std::vector<JointState>::const_iterator iter =
       this->jointStates.begin(); iter != this->jointStates.end(); ++iter)
  {
    try
    {
      JointState state = (*iter) - _state.GetJointState((*iter).GetName());
      if (!state.IsZero())
        result.jointStates.push_back(state);
    }
    catch(common::Exception &)
    {
      // Ignore exception, which is just the fact that a joint state may not
      // have been recorded.
    }
  }

  return result;
}

/////////////////////////////////////////////////
ModelState ModelState::operator+(const ModelState &_state) const
{
  ModelState result;

  result.name = this->name;
  result.pose.pos = this->pose.pos + _state.pose.pos;
  result.pose.rot = _state.pose.rot * this->pose.rot;

  // Insert the link state diffs.
  for (std::vector<LinkState>::const_iterator iter =
       this->linkStates.begin(); iter != this->linkStates.end(); ++iter)
  {
    try
    {
      LinkState state = (*iter) + _state.GetLinkState((*iter).GetName());
      result.linkStates.push_back(state);
    }
    catch(common::Exception &)
    {
      // Ignore exception, which is just the fact that a link state may not
      // have been recorded.
    }
  }

  // Insert the joint state diffs.
  for (std::vector<JointState>::const_iterator iter =
       this->jointStates.begin(); iter != this->jointStates.end(); ++iter)
  {
    try
    {
      JointState state = (*iter) + _state.GetJointState((*iter).GetName());
      result.jointStates.push_back(state);
    }
    catch(common::Exception &)
    {
      // Ignore exception, which is just the fact that a joint state may not
      // have been recorded.
    }
  }

  return result;
}

/////////////////////////////////////////////////
void ModelState::FillSDF(sdf::ElementPtr _sdf)
{
  _sdf->ClearElements();

  _sdf->GetAttribute("name")->Set(this->name);
  _sdf->GetElement("pose")->Set(this->pose);

  for (std::vector<LinkState>::iterator iter = this->linkStates.begin();
       iter != this->linkStates.end(); ++iter)
  {
    sdf::ElementPtr elem = _sdf->AddElement("link");
    (*iter).FillSDF(elem);
  }

  for (std::vector<JointState>::iterator iter = this->jointStates.begin();
       iter != this->jointStates.end(); ++iter)
  {
    sdf::ElementPtr elem = _sdf->AddElement("joint");
    (*iter).FillSDF(elem);
  }
}
