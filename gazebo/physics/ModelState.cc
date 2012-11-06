/*
 * Copyright 2011 Nate Koenig
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

#include "physics/Model.hh"
#include "physics/Link.hh"
#include "physics/World.hh"
#include "physics/ModelState.hh"

using namespace gazebo;
using namespace physics;

/////////////////////////////////////////////////
ModelState::ModelState()
: State()
{
}

/////////////////////////////////////////////////
ModelState::ModelState(ModelPtr _model)
: State(_model->GetName(), _model->GetWorld()->GetRealTime(),
        _model->GetWorld()->GetSimTime())
{
  this->pose = _model->GetWorldPose();

  const Link_V links = _model->GetLinks();
  for (Link_V::const_iterator iter = links.begin(); iter != links.end(); ++iter)
  {
    this->linkStates.push_back(LinkState((*iter)));
  }

  const Joint_V joints = _model->GetJoints();
  for (Joint_V::const_iterator iter = joints.begin();
       iter != joints.end(); ++iter)
  {
    this->jointStates.push_back(JointState(*iter));
  }
}

/////////////////////////////////////////////////
ModelState::~ModelState()
{
}

/////////////////////////////////////////////////
void ModelState::Load(sdf::ElementPtr _elem)
{
  this->name = _elem->GetValueString("name");

  if (_elem->HasElement("pose"))
    this->pose = _elem->GetValuePose("pose");

  if (_elem->HasElement("link"))
  {
    sdf::ElementPtr childElem = _elem->GetElement("link");

    while (childElem)
    {
      LinkState state;
      state.Load(childElem);
      this->linkStates.push_back(state);
      childElem = childElem->GetNextElement("link");
    }
  }
}

/////////////////////////////////////////////////
void ModelState::FillStateSDF(sdf::ElementPtr _elem) const
{
  _elem->GetAttribute("name")->Set(this->GetName());
  _elem->GetElement("pose")->GetValue()->Set(this->pose);

  for (std::vector<LinkState>::const_iterator iter = this->linkStates.begin();
       iter != this->linkStates.end(); ++iter)
  {
    sdf::ElementPtr elem = _elem->AddElement("link");
    (*iter).FillStateSDF(elem);
  }
}

/////////////////////////////////////////////////
void ModelState::UpdateModelSDF(sdf::ElementPtr _elem)
{
  _elem->GetElement("pose")->Set(this->pose);

  if (_elem->HasElement("link"))
  {
    sdf::ElementPtr childElem = _elem->GetElement("link");

    // Update all links
    while (childElem)
    {
      // Find matching link state
      for (std::vector<LinkState>::iterator iter = this->linkStates.begin();
          iter != this->linkStates.end(); ++iter)
      {
        if ((*iter).GetName() == childElem->GetValueString("name"))
        {
          (*iter).UpdateLinkSDF(childElem);
        }
      }

      childElem = childElem->GetNextElement("link");
    }
  }
}

/////////////////////////////////////////////////
math::Pose ModelState::GetPose() const
{
  return this->pose;
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
  else
    gzerr << "Index is out of range\n";

  return LinkState();
}

/////////////////////////////////////////////////
LinkState ModelState::GetLinkState(const std::string &_linkName) const
{
  std::vector<LinkState>::const_iterator iter;
  for (iter = this->linkStates.begin(); iter != this->linkStates.end(); ++iter)
  {
    if ((*iter).GetName() == _linkName)
      return *iter;
  }

  return LinkState();
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
  else
    gzerr << "Index is out of range\n";

  return JointState();
}

/////////////////////////////////////////////////
JointState ModelState::GetJointState(const std::string &_jointName) const
{
  std::vector<JointState>::const_iterator iter;
  for (iter = this->jointStates.begin();
       iter != this->jointStates.end(); ++iter)
  {
    if ((*iter).GetName() == _jointName)
      return *iter;
  }

  return JointState();
}

/////////////////////////////////////////////////
bool ModelState::IsZero() const
{
  bool result = true;

  for (std::vector<LinkState>::const_iterator iter =
       this->linkStates.begin();
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

  result = *this;

  result.pose = this->pose - _state.pose;

  result.linkStates.clear();
  result.jointStates.clear();

  // Insert the link state diffs.
  for (std::vector<LinkState>::const_iterator iter =
       this->linkStates.begin(); iter != this->linkStates.end(); ++iter)
  {
    LinkState state = (*iter) - _state.GetLinkState((*iter).GetName());
    if (!state.IsZero())
      result.linkStates.push_back(state);
  }

  // Insert the joint state diffs.
  for (std::vector<JointState>::const_iterator iter =
       this->jointStates.begin(); iter != this->jointStates.end(); ++iter)
  {
    JointState state = (*iter) - _state.GetJointState((*iter).GetName());
    if (!state.IsZero())
      result.jointStates.push_back(state);
  }

  return result;
}

/////////////////////////////////////////////////
ModelState ModelState::operator+(const ModelState &_state) const
{
  ModelState result;

  result = *this;

  result.pose = this->pose + _state.pose;

  result.linkStates.clear();
  result.jointStates.clear();

  // Insert the link state diffs.
  /*for (std::vector<LinkState>::const_iterator iter =
       this->linkStates.begin(); iter != this->linkStates.end(); ++iter)
  {
    LinkState state = (*iter) + _state.GetLinkState((*iter).GetName());
    result.linkStates.push_back(state);
  }

  // Insert the joint state diffs.
  for (std::vector<JointState>::const_iterator iter =
       this->jointStates.begin(); iter != this->jointStates.end(); ++iter)
  {
    JointState state = (*iter) + _state.GetJointState((*iter).GetName());
    result.jointStates.push_back(state);
  }
  */
  return result;
}
