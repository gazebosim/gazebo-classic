/*
 * Copyright 2012 Open Source Robotics Foundation
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
#include "transport/ConnectionManager.hh"
#include "transport/SubscriptionTransport.hh"

using namespace gazebo;
using namespace transport;

//////////////////////////////////////////////////
SubscriptionTransport::SubscriptionTransport()
{
}

//////////////////////////////////////////////////
SubscriptionTransport::~SubscriptionTransport()
{
  ConnectionManager::Instance()->RemoveConnection(this->connection);
}

//////////////////////////////////////////////////
void SubscriptionTransport::Init(const ConnectionPtr &_conn, bool _latching)
{
  this->connection = _conn;
  this->latching = _latching;
}

//////////////////////////////////////////////////
bool SubscriptionTransport::HandleData(const std::string &newdata)
{
  bool result = false;
  if (this->connection->IsOpen())
  {
    this->connection->EnqueueMsg(newdata);
    result = true;
  }
  else
    this->connection.reset();

  return result;
}

//////////////////////////////////////////////////
const ConnectionPtr &SubscriptionTransport::GetConnection() const
{
  return this->connection;
}

//////////////////////////////////////////////////
/// remote connection
bool SubscriptionTransport::IsLocal() const
{
  return false;
}

