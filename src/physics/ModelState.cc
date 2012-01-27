/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
    // this->linkStates.push_back(_model->GetLink(i)->GetState());
  }
  this->pose = _model->GetWorldPose();
}

/////////////////////////////////////////////////
ModelState::~ModelState()
{
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
