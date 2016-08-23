/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
ModelState::ModelState(const ModelPtr _model, const common::Time &_realTime,
    const common::Time &_simTime, const uint64_t _iterations)
: State(_model->GetName(), _realTime, _simTime, _iterations)
{
  this->pose = _model->GetWorldPose();

  // Copy all the links
  const Link_V links = _model->GetLinks();
  for (Link_V::const_iterator iter = links.begin(); iter != links.end(); ++iter)
  {
    this->linkStates.insert(std::make_pair((*iter)->GetName(),
          LinkState(*iter, _realTime, _simTime, _iterations)));
  }

  // Copy all the joints
  /*const Joint_V joints = _model->GetJoints();
  for (Joint_V::const_iterator iter = joints.begin();
       iter != joints.end(); ++iter)
  {
    this->jointStates.insert(std::make_pair((*iter)->GetName(),
          JointState(*iter, _realTime, _simTime, _iterations)));
  }*/
}

/////////////////////////////////////////////////
ModelState::ModelState(const ModelPtr _model)
: State(_model->GetName(), _model->GetWorld()->GetRealTime(),
        _model->GetWorld()->GetSimTime(), _model->GetWorld()->GetIterations())
{
  this->pose = _model->GetWorldPose();

  // Copy all the links
  const Link_V links = _model->GetLinks();
  for (Link_V::const_iterator iter = links.begin(); iter != links.end(); ++iter)
  {
    this->linkStates.insert(std::make_pair((*iter)->GetName(),
          LinkState((*iter))));
  }

  // Copy all the joints
  /*const Joint_V joints = _model->GetJoints();
  for (Joint_V::const_iterator iter = joints.begin();
       iter != joints.end(); ++iter)
  {
    this->jointStates.insert(std::make_pair((*iter)->GetName(),
          JointState(*iter)));
  }*/
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
void ModelState::Load(const ModelPtr _model, const common::Time &_realTime,
    const common::Time &_simTime, const uint64_t _iterations)
{
  this->name = _model->GetName();
  this->wallTime = common::Time::GetWallTime();
  this->realTime = _realTime;
  this->simTime = _simTime;
  this->iterations = _iterations;
  this->pose = _model->GetWorldPose();

  // Load all the links
  const Link_V links = _model->GetLinks();
  for (Link_V::const_iterator iter = links.begin(); iter != links.end(); ++iter)
  {
    this->linkStates[(*iter)->GetName()].Load(*iter, _realTime, _simTime,
        _iterations);
  }

  // Remove links that no longer exist. We determine this by check the time
  // stamp on each link.
  for (LinkState_M::iterator iter = this->linkStates.begin();
       iter != this->linkStates.end();)
  {
    if (iter->second.GetRealTime() != this->realTime)
      this->linkStates.erase(iter++);
    else
      ++iter;
  }

  // Copy all the joints
  /*const Joint_V joints = _model->GetJoints();
  for (Joint_V::const_iterator iter = joints.begin();
      iter != joints.end(); ++iter)
  {
    this->jointStates[(*iter)->GetName()].Load(*iter, _realTime, _simTime,
        _iterations);
  }

  // Remove joints that no longer exist. We determine this by check the time
  // stamp on each joint.
  for (JointState_M::iterator iter = this->jointStates.begin();
       iter != this->jointStates.end();)
  {
    if (iter->second.GetRealTime() != this->realTime)
      this->jointStates.erase(iter++);
    else
      ++iter;
  }*/
}

/////////////////////////////////////////////////
void ModelState::Load(const sdf::ElementPtr _elem)
{
  // Set the name
  this->name = _elem->Get<std::string>("name");

  // Set the model pose
  if (_elem->HasElement("pose"))
    this->pose = _elem->Get<math::Pose>("pose");
  else
    this->pose.Set(0, 0, 0, 0, 0, 0);

  // Set all the links
  this->linkStates.clear();
  if (_elem->HasElement("link"))
  {
    sdf::ElementPtr childElem = _elem->GetElement("link");

    while (childElem)
    {
      this->linkStates.insert(std::make_pair(
            childElem->Get<std::string>("name"), LinkState(childElem)));
      childElem = childElem->GetNextElement("link");
    }
  }

  // Set all the joints
  /*this->jointStates.clear();
  if (_elem->HasElement("joint"))
  {
    sdf::ElementPtr childElem = _elem->GetElement("joint");

    while (childElem)
    {
      this->jointStates.insert(std::make_pair(childElem->Get<std::string>("name"),
            JointState(childElem)));
      childElem = childElem->GetNextElement("joint");
    }
  }*/
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

  for (LinkState_M::const_iterator iter = this->linkStates.begin();
       iter != this->linkStates.end() && result; ++iter)
  {
    result = result && iter->second.IsZero();
  }

  /*for (JointState_M::const_iterator iter = this->jointStates.begin();
       iter != this->jointStates.end() && result; ++iter)
  {
    result = result && iter->second.IsZero();
  }*/

  return result && this->pose == math::Pose::Zero;
}

/////////////////////////////////////////////////
unsigned int ModelState::GetLinkStateCount() const
{
  return this->linkStates.size();
}

/////////////////////////////////////////////////
LinkState_M ModelState::GetLinkStates(const boost::regex &_regex) const
{
  LinkState_M result;

  // Search for matching link names
  for (LinkState_M::const_iterator iter = this->linkStates.begin();
       iter != this->linkStates.end(); ++iter)
  {
    if (boost::regex_match(iter->first, _regex))
      result.insert(std::make_pair(iter->first, iter->second));
  }

  return result;
}

/////////////////////////////////////////////////
JointState_M ModelState::GetJointStates(const boost::regex &_regex) const
{
  JointState_M result;

  // Search for matching link names
  for (JointState_M::const_iterator iter = this->jointStates.begin();
       iter != this->jointStates.end(); ++iter)
  {
    if (boost::regex_match(iter->second.GetName(), _regex))
      result.insert(std::make_pair(iter->first, iter->second));
  }

  return result;
}

/////////////////////////////////////////////////
LinkState ModelState::GetLinkState(const std::string &_linkName) const
{
  // Search for the link name
  LinkState_M::const_iterator iter = this->linkStates.find(_linkName);
  if (iter != this->linkStates.end())
    return iter->second;

  gzthrow("Invalid link name[" + _linkName + "]");
  return LinkState();
}

/////////////////////////////////////////////////
bool ModelState::HasLinkState(const std::string &_linkName) const
{
  // Search for the link name
  LinkState_M::const_iterator iter = this->linkStates.find(_linkName);
  if (iter != this->linkStates.end())
    return true;

  return false;
}

/////////////////////////////////////////////////
const LinkState_M &ModelState::GetLinkStates() const
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
  {
    JointState_M::const_iterator iter = this->jointStates.begin();
    std::advance(iter, _index);
    return iter->second;
  }

  gzthrow("Index is out of range");
  return JointState();
}

/////////////////////////////////////////////////
JointState ModelState::GetJointState(const std::string &_jointName) const
{
  JointState_M::const_iterator iter = this->jointStates.find(_jointName);
  if (iter != this->jointStates.end())
    return iter->second;

  gzthrow("Invalid joint name[" + _jointName + "]");
  return JointState();
}

/////////////////////////////////////////////////
bool ModelState::HasJointState(const std::string &_jointName) const
{
  JointState_M::const_iterator iter = this->jointStates.find(_jointName);
  if (iter != this->jointStates.end())
    return true;


  return false;
}

/////////////////////////////////////////////////
const JointState_M &ModelState::GetJointStates() const
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
  for (LinkState_M::const_iterator iter =
       _state.linkStates.begin(); iter != _state.linkStates.end(); ++iter)
  {
    this->linkStates.insert(std::make_pair(iter->first, iter->second));
  }

  // Copy the joint states.
  // for (JointState_M::const_iterator iter =
  //     _state.jointStates.begin(); iter != _state.jointStates.end(); ++iter)
  // {
  //   this->jointStates.insert(std::make_pair(iter->first, iter->second));
  // }

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
  for (LinkState_M::const_iterator iter =
       this->linkStates.begin(); iter != this->linkStates.end(); ++iter)
  {
    try
    {
      if (_state.HasLinkState(iter->second.GetName()))
      {
        LinkState state = iter->second - _state.GetLinkState(
            iter->second.GetName());
        if (!state.IsZero())
          result.linkStates.insert(std::make_pair(state.GetName(), state));
      }
    }
    catch(common::Exception &)
    {
      // Ignore exception, which is just the fact that a link state may not
      // have been recorded.
    }
  }

  // Insert the joint state diffs.
  /*for (JointState_M::const_iterator iter =
       this->jointStates.begin(); iter != this->jointStates.end(); ++iter)
  {
    try
    {
      if (_state.HasJointState(iter->second.GetName()))
      {
        JointState state = iter->second -
          _state.GetJointState(iter->second.GetName());
        if (!state.IsZero())
          result.jointStates.insert(std::make_pair(state.GetName(), state));
      }
    }
    catch(common::Exception &)
    {
      // Ignore exception, which is just the fact that a joint state may not
      // have been recorded.
    }
  }*/

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
  for (LinkState_M::const_iterator iter =
       this->linkStates.begin(); iter != this->linkStates.end(); ++iter)
  {
    try
    {
      if (_state.HasLinkState(iter->second.GetName()))
      {
        LinkState state = iter->second + _state.GetLinkState(
            iter->second.GetName());
        result.linkStates.insert(std::make_pair(state.GetName(), state));
      }
    }
    catch(common::Exception &)
    {
      // Ignore exception, which is just the fact that a link state may not
      // have been recorded.
    }
  }

  // Insert the joint state diffs.
  for (JointState_M::const_iterator iter =
       this->jointStates.begin(); iter != this->jointStates.end(); ++iter)
  {
    try
    {
      if (_state.HasJointState(iter->second.GetName()))
      {
        JointState state = iter->second +
          _state.GetJointState(iter->second.GetName());
        result.jointStates.insert(std::make_pair(state.GetName(), state));
      }
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

  for (LinkState_M::iterator iter = this->linkStates.begin();
       iter != this->linkStates.end(); ++iter)
  {
    sdf::ElementPtr elem = _sdf->AddElement("link");
    iter->second.FillSDF(elem);
  }

  for (JointState_M::iterator iter = this->jointStates.begin();
       iter != this->jointStates.end(); ++iter)
  {
    sdf::ElementPtr elem = _sdf->AddElement("joint");
    iter->second.FillSDF(elem);
  }
}

/////////////////////////////////////////////////
void ModelState::SetWallTime(const common::Time &_time)
{
  State::SetWallTime(_time);

  for (LinkState_M::iterator iter = this->linkStates.begin();
       iter != this->linkStates.end(); ++iter)
  {
    iter->second.SetWallTime(_time);
  }

  for (JointState_M::iterator iter = this->jointStates.begin();
       iter != this->jointStates.end(); ++iter)
  {
    iter->second.SetWallTime(_time);
  }
}

/////////////////////////////////////////////////
void ModelState::SetRealTime(const common::Time &_time)
{
  State::SetRealTime(_time);

  for (LinkState_M::iterator iter = this->linkStates.begin();
           iter != this->linkStates.end(); ++iter)
  {
    iter->second.SetRealTime(_time);
  }

  for (JointState_M::iterator iter = this->jointStates.begin();
       iter != this->jointStates.end(); ++iter)
  {
    iter->second.SetRealTime(_time);
  }
}

/////////////////////////////////////////////////
void ModelState::SetSimTime(const common::Time &_time)
{
  State::SetSimTime(_time);

  for (LinkState_M::iterator iter = this->linkStates.begin();
       iter != this->linkStates.end(); ++iter)
  {
    iter->second.SetSimTime(_time);
  }

  for (JointState_M::iterator iter = this->jointStates.begin();
       iter != this->jointStates.end(); ++iter)
  {
    iter->second.SetSimTime(_time);
  }
}

/////////////////////////////////////////////////
void ModelState::SetIterations(const uint64_t _iterations)
{
  State::SetIterations(_iterations);

  for (auto &linkState : this->linkStates)
    linkState.second.SetIterations(_iterations);

  for (auto &jointState : this->jointStates)
    jointState.second.SetIterations(_iterations);
}
