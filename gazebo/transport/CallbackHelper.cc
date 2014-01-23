/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

#include "gazebo/transport/CallbackHelper.hh"

using namespace gazebo;
using namespace transport;

unsigned int CallbackHelper::idCounter = 0;

/////////////////////////////////////////////////
CallbackHelper::CallbackHelper(bool _latching)
  : latching(_latching), id(idCounter++)
{
}

/////////////////////////////////////////////////
CallbackHelper::~CallbackHelper()
{
}

/////////////////////////////////////////////////
std::string CallbackHelper::GetMsgType() const
{
  return std::string();
}

/////////////////////////////////////////////////
bool CallbackHelper::GetLatching() const
{
  return this->latching;
}

/////////////////////////////////////////////////
unsigned int CallbackHelper::GetId() const
{
  return this->id;
}
