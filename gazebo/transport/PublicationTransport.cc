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

#include "gazebo/transport/TopicManager.hh"
#include "gazebo/transport/ConnectionManager.hh"
#include "gazebo/transport/PublicationTransport.hh"

using namespace gazebo;
using namespace transport;

int PublicationTransport::counter = 0;

/////////////////////////////////////////////////
PublicationTransport::PublicationTransport(const std::string &_topic,
                                           const std::string &_msgType)
: topic(_topic), msgType(_msgType)
{
  this->id = counter++;
  TopicManager::Instance()->UpdatePublications(this->topic, this->msgType);
}

/////////////////////////////////////////////////
PublicationTransport::~PublicationTransport()
{
  if (this->connection)
  {
    msgs::Subscribe sub;
    sub.set_topic(this->topic);
    sub.set_msg_type(this->msgType);
    sub.set_host(this->connection->GetLocalAddress());
    sub.set_port(this->connection->GetLocalPort());
    ConnectionManager::Instance()->Unsubscribe(sub);
    this->connection->Cancel();
    this->connection.reset();

    ConnectionManager::Instance()->RemoveConnection(this->connection);
  }
  this->callback.clear();
}

/////////////////////////////////////////////////
void PublicationTransport::Init(const ConnectionPtr &_conn, bool _latched)
{
  this->connection = _conn;
  msgs::Subscribe sub;
  sub.set_topic(this->topic);
  sub.set_msg_type(this->msgType);
  sub.set_host(this->connection->GetLocalAddress());
  sub.set_port(this->connection->GetLocalPort());
  sub.set_latching(_latched);

  this->connection->EnqueueMsg(msgs::Package("sub", sub));

  // Put this in PublicationTransportPtr
  // Start reading messages from the remote publisher
  this->connection->AsyncRead(boost::bind(&PublicationTransport::OnPublish,
        this, _1));
}


/////////////////////////////////////////////////
void PublicationTransport::AddCallback(
    const boost::function<void(const std::string &)> &cb_)
{
  this->callback = cb_;
}

/////////////////////////////////////////////////
void PublicationTransport::OnPublish(const std::string &_data)
{
  if (this->connection && this->connection->IsOpen())
  {
    this->connection->AsyncRead(
        boost::bind(&PublicationTransport::OnPublish, this, _1));

    if (!_data.empty())
    {
      if (this->callback)
        (this->callback)(_data);
    }
  }
}

/////////////////////////////////////////////////
const ConnectionPtr PublicationTransport::GetConnection() const
{
  return this->connection;
}

/////////////////////////////////////////////////
std::string PublicationTransport::GetTopic() const
{
  return this->topic;
}

/////////////////////////////////////////////////
std::string PublicationTransport::GetMsgType() const
{
  return this->msgType;
}

/////////////////////////////////////////////////
void PublicationTransport::Fini()
{
  /// Cancel all async operatiopns.
  if (this->connection)
  {
    this->connection->Cancel();
    // this->connection.reset();
  }
}
