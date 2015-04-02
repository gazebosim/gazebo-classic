/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
#include "gazebo/transport/ConnectionManager.hh"
#include "gazebo/transport/SubscriptionTransport.hh"

using namespace gazebo;
using namespace transport;

extern void dummy_callback_fn(uint32_t);

//////////////////////////////////////////////////
SubscriptionTransport::SubscriptionTransport()
{
}

//////////////////////////////////////////////////
SubscriptionTransport::~SubscriptionTransport()
{
  ConnectionManager::Instance()->RemoveConnection(this->connection);
  this->connection.reset();
}

//////////////////////////////////////////////////
void SubscriptionTransport::Init(ConnectionPtr _conn, bool _latching)
{
  this->connection = _conn;
  this->latching = _latching;
}

//////////////////////////////////////////////////
bool SubscriptionTransport::HandleMessage(MessagePtr _newMsg)
{
  std::string data;
  _newMsg->SerializeToString(&data);
  return this->HandleData(data, boost::bind(&dummy_callback_fn, _1), 0);
}

//////////////////////////////////////////////////
bool SubscriptionTransport::HandleData(const std::string &_newdata,
    boost::function<void(uint32_t)> _cb, uint32_t _id)
{
  bool result = false;
  if (this->connection->IsOpen())
  {
    this->connection->EnqueueMsg(_newdata, _cb, _id);
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
