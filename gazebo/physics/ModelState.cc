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
  for (unsigned int i = 0; i < _model->GetChildCount(); ++i)
  {
    this->linkStates.push_back(_model->GetLink(i)->GetState());
  }

  /*for (unsigned int i = 0; i < _model->GetJointCount(); ++i)
  {
    this->jointStates.push_back(_model->GetJoint(i)->GetState());
  }*/

  this->pose = _model->GetWorldPose();
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
    this->pose = _elem->GetElement("pose")->GetValuePose("");

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
void ModelState::FillStateSDF(sdf::ElementPtr _elem)
{
  _elem->GetAttribute("name")->Set(this->GetName());
  _elem->GetElement("pose")->GetValue()->Set(this->pose);

  for (std::vector<LinkState>::iterator iter = this->linkStates.begin();
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
