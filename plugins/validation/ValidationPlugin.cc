/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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

#include <functional>
#include <mutex>
#include <string>
#include <ignition/math/Helpers.hh>
#include "gazebo/common/Console.hh"
#include "State.hh"
#include "ValidationPlugin.hh"

using namespace gazebo;

//////////////////////////////////////////////////
GazeboState::GazeboState(const std::string &_name,
    ValidationPlugin &_plugin)
  : State(_name, ValidationComponent_t::GAZEBO),
    plugin(_plugin)
{
}

//////////////////////////////////////////////////
void GazeboReadyState::DoFeedback()
{
  std::lock_guard<std::mutex> lock(this->mutex);
  if (*this->plugin.currentState != *this)
    return;

  // Is the controller ready?
  if (this->Feedback() == "controller_ready")
    this->plugin.ChangeState(*this->plugin.setState);

  //std::cout << "ReadyState::DoFeedback()" << std::endl;
}

//////////////////////////////////////////////////
void GazeboSetState::DoInitialize()
{
  std::lock_guard<std::mutex> lock(this->mutex);

  // Load the parameters.
  if (!this->plugin.LoadModelParams())
  {
    this->plugin.ChangeState(*this->plugin.readyState);
    return;
  }

  // Start the run.
  this->plugin.ChangeState(*this->plugin.goState);

  std::cout << "SetState::DoInitialize()" << std::endl;
}

//////////////////////////////////////////////////
void GazeboGoState::DoInitialize()
{
  std::lock_guard<std::mutex> lock(this->mutex);
  std::cout << "GoState::DoInitialize()" << std::endl;
}

//////////////////////////////////////////////////
void GazeboGoState::DoFeedback()
{
  std::lock_guard<std::mutex> lock(this->mutex);
  if (this->Feedback() == "controller_end")
  {
    if (this->plugin.MoreRuns())
    {
      // Go for the next run.
      this->plugin.ChangeState(*this->plugin.readyState);
    }
    else
    {
      this->plugin.ChangeState(*this->plugin.endState);
    }
  }
}

//////////////////////////////////////////////////
void GazeboEndState::DoInitialize()
{
  std::lock_guard<std::mutex> lock(this->mutex);
  std::cout << "EndState::DoInitialize()" << std::endl;
}

//////////////////////////////////////////////////
bool ParseValidationBlock(sdf::ElementPtr _sdf,
    std::string &_name, double &_min, double &_max, double &_step)
{
  if (!_sdf->HasElement("name")||!_sdf->HasElement("min") ||
      !_sdf->HasElement("max") ||!_sdf->HasElement("step"))
    return false;

  _name = _sdf->Get<std::string>("name");
  _min = _sdf->Get<double>("min");
  _max = _sdf->Get<double>("max");
  _step = _sdf->Get<double>("step");

  if (_max < _min)
  {
    std::cerr << "<max> [" << _max << "] is lower than <min> [" << _min << "]"
              << ". This doesn't make sense." << std::endl;
    return false;
  }

  return true;
}

//////////////////////////////////////////////////
bool FillVector(const std::string &_name, const double _min, const double _max,
    const double _step, std::map<std::string, std::vector<double>> &_m)
{
  if (_m.find(_name) != _m.end())
  {
    std::cerr << "Joint [" << _name << "] already registered for this parameter"
              << std::endl;
    return false;
  }
  double current = _min;
  while (current <= _max)
  {
    _m[_name].push_back(current);
    current += _step;
  }

  if (!ignition::math::equal(_m[_name].back(),_max))
    _m[_name].push_back(_max);

  return true;
}

//////////////////////////////////////////////////
ValidationPlugin::ValidationPlugin()
  : readyState(new GazeboReadyState(kReadyState, *this)),
    setState(new GazeboSetState(kSetState, *this)),
    goState(new GazeboGoState(kGoState, *this)),
    endState(new GazeboEndState(kEndState, *this)),
    currentState(readyState.get())
{
  std::cout << "Validation plugin loaded" << std::endl;
}

//////////////////////////////////////////////////
void ValidationPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "ValidationPlugin _model pointer is NULL");
  this->model = _model;
  GZ_ASSERT(_sdf, "ValidationPlugin _sdf pointer is NULL");

  std::string name;
  double min, max, step;

  sdf::ElementPtr elem = _sdf->GetFirstElement();
  while (elem)
  {
    if (elem->GetName() == "lower_limit")
    {
      // Read the <lower_limit> element.
      auto lowerLimitElem = _sdf->GetElement("lower_limit");
      if (!ParseValidationBlock(lowerLimitElem, name, min, max, step))
      {
        std::cerr << "Error parsing [lower_limit] params" << std::endl;
        return;
      }

      if (!FillVector(name, min, max, step, this->lowerLimitParams))
        return;
    }
    else if (elem->GetName() == "upper_limit")
    {
      // Read the <upper_limit> element.
      auto upperLimitElem = _sdf->GetElement("upper_limit");
      if (!ParseValidationBlock(upperLimitElem, name, min, max, step))
      {
        std::cerr << "Error parsing [lower_limit] params" << std::endl;
        return;
      }

      if (!FillVector(name, min, max, step, this->upperLimitParams))
        return;
    }
    elem = elem->GetNextElement();
  }

  // Parameters to test:
  //std::cout << "Lower limits" << std::endl;
  //for (auto v : this->lowerLimitParams)
  //  std::cout << v << std::endl;

  //std::cout << "Upper limits" << std::endl;
  //for (auto v : this->upperLimitParams)
  //  std::cout << v << std::endl;

  // Set up a physics update callback
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    std::bind(&ValidationPlugin::WorldUpdateCallback, this));
}

//////////////////////////////////////////////////
void ValidationPlugin::WorldUpdateCallback()
{
  if (this->currentState)
    this->currentState->Update();
}

//////////////////////////////////////////////////
bool ValidationPlugin::LoadModelParams()
{
  if (!this->lowerLimitParams.empty())
  {
    std::string name = this->lowerLimitParams.begin()->first;
    double value = this->lowerLimitParams.begin()->second.front();

    this->lowerLimitParams.begin()->second.erase(
      this->lowerLimitParams.begin()->second.begin());
    if (this->lowerLimitParams.begin()->second.empty())
      this->lowerLimitParams.erase(this->lowerLimitParams.begin());

    auto joint = this->model->GetJoint(name);
    if (!joint)
    {
      std::cerr << "Unable to find joint [" << name << "]" << std::endl;
      return false;
    }

    std::cout << "Setting [" << value << "] as the lower joint limit on "
              << name << std::endl;

    joint->SetLowerLimit(0, gazebo::math::Angle(value));
    return true;
  }

  if (!this->upperLimitParams.empty())
  {
    std::string name = this->upperLimitParams.begin()->first;
    double value = this->upperLimitParams.begin()->second.front();

    this->upperLimitParams.begin()->second.erase(
      this->upperLimitParams.begin()->second.begin());
    if (this->upperLimitParams.begin()->second.empty())
      this->upperLimitParams.erase(this->upperLimitParams.begin());

    auto joint = this->model->GetJoint(name);
    if (!joint)
    {
      std::cerr << "Unable to find joint [" << name << "]" << std::endl;
      return false;
    }

    std::cout << "Setting [" << value << "] as the upper joint limit on "
              << name << std::endl;
    joint->SetUpperLimit(0, gazebo::math::Angle(value));
    return true;
  }

  return false;
}

//////////////////////////////////////////////////
void ValidationPlugin::ChangeState(State &_newState)
{
  // Only update the state if _newState is different than the current state.
  if (!this->currentState || *this->currentState != _newState)
  {
    this->currentState->Teardown();
    this->currentState = &_newState;
  }
}

//////////////////////////////////////////////////
bool ValidationPlugin::MoreRuns() const
{
  static int counter = 0;
  ++counter;

  return counter <= 3;
}
