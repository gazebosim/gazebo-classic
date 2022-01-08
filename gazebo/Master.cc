/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
#include <thread>
#include <mutex>

#include <boost/bind/bind.hpp>
#include <boost/make_shared.hpp>
#include <google/protobuf/descriptor.h>
#include <set>
#include "gazebo/transport/IOManager.hh"

#include "Master.hh"

#include "gazebo/gazebo_config.h"

using namespace gazebo;

namespace gazebo
{
  struct MasterPrivate
  {
    /// \brief All the known publishers.
    gazebo::Master::PubList publishers;

    /// \brief All the known subscribers.
    gazebo::Master::SubList subscribers;

    /// \brief All the known connections.
    gazebo::Master::Connection_M connections;

    /// \brief All the worlds.
    std::list<std::string> worldNames;

    /// \brief Incoming messages.
    std::list<std::pair<unsigned int, std::string> > msgs;

    /// \brief Our server connection.
    transport::ConnectionPtr connection;

    /// \brief Thread to run the main loop.
    std::thread *runThread;

    /// \brief True to stop Master.
    bool stop;

    /// \brief Mutex to protect connections.
    std::recursive_mutex connectionMutex;

    /// \brief Mutex to protect msg bufferes.
    std::recursive_mutex msgsMutex;
  };
}

/////////////////////////////////////////////////
Master::Master()
  : dataPtr(new MasterPrivate())
{
  this->dataPtr->stop = false;
  this->dataPtr->runThread = NULL;
  this->dataPtr->connection = boost::make_shared<transport::Connection>();
}

/////////////////////////////////////////////////
Master::~Master()
{
  this->Fini();
}

/////////////////////////////////////////////////
void Master::Init(uint16_t _port)
{
  try
  {
    using namespace boost::placeholders;
    this->dataPtr->connection->Listen(_port,
          boost::bind(&Master::OnAccept, this, _1));
  }
  catch(std::exception &_e)
  {
    gzthrow("Unable to start server[" << _e.what() << "]. "
            "There is probably another Gazebo process running.");
  }
}

//////////////////////////////////////////////////
void Master::OnAccept(transport::ConnectionPtr _newConnection)
{
  // Send the gazebo version string
  msgs::GzString versionMsg;
  versionMsg.set_data(std::string("gazebo ") + GAZEBO_VERSION);
  _newConnection->EnqueueMsg(msgs::Package("version_init", versionMsg), true);

  // Send all the current topic namespaces
  msgs::GzString_V namespacesMsg;
  std::list<std::string>::iterator iter;
  for (iter = this->dataPtr->worldNames.begin();
       iter != this->dataPtr->worldNames.end(); ++iter)
  {
    namespacesMsg.add_data(*iter);
  }
  _newConnection->EnqueueMsg(msgs::Package("topic_namepaces_init",
                              namespacesMsg), true);

  // Send all the publishers
  msgs::Publishers publishersMsg;
  PubList::iterator pubiter;
  for (pubiter = this->dataPtr->publishers.begin();
       pubiter != this->dataPtr->publishers.end(); ++pubiter)
  {
    msgs::Publish *pub = publishersMsg.add_publisher();
    pub->CopyFrom(pubiter->first);
  }
  _newConnection->EnqueueMsg(
      msgs::Package("publishers_init", publishersMsg), true);

  // Add the connection to our list
  {
    std::lock_guard<std::recursive_mutex> lock(this->dataPtr->connectionMutex);
    int index = this->dataPtr->connections.size();

    this->dataPtr->connections[index] = _newConnection;

    // Start reading from the connection
    using namespace boost::placeholders;
    _newConnection->AsyncRead(
        boost::bind(&Master::OnRead, this, index, _1));
  }
}

//////////////////////////////////////////////////
void Master::OnRead(const unsigned int _connectionIndex,
                    const std::string &_data)
{
  if (this->dataPtr->stop)
    return;

  if (!this->dataPtr->connections[_connectionIndex] ||
      !this->dataPtr->connections[_connectionIndex]->IsOpen())
    return;

  // Get the connection
  transport::ConnectionPtr conn = this->dataPtr->connections[_connectionIndex];

  // Read the next message
  using namespace boost::placeholders;
  if (conn && conn->IsOpen())
    conn->AsyncRead(boost::bind(&Master::OnRead, this, _connectionIndex, _1));

  // Store the message if it's not empty
  if (!_data.empty())
  {
    std::lock_guard<std::recursive_mutex> lock(this->dataPtr->msgsMutex);
    this->dataPtr->msgs.push_back(std::make_pair(_connectionIndex, _data));
  }
  else
  {
    gzlog << "Master got empty data message from["
          << conn->GetRemotePort() << "]. This is most likely fine, since"
          << "the remote side probably terminated.\n";
  }
}

//////////////////////////////////////////////////
void Master::SendSubscribers(const std::string &_topic,
                             const std::string &_buffer)
{
  // Find all subscribers for this topic
  std::set<transport::ConnectionPtr> uniqueConnections;
  for (auto const &subscriber : this->dataPtr->subscribers)
    if (subscriber.first.topic() == _topic)
      uniqueConnections.insert(subscriber.second);

  // Send message to all unique connections
  for (auto &conn : uniqueConnections)
    conn->EnqueueMsg(_buffer);
}

//////////////////////////////////////////////////
void Master::ProcessMessage(const unsigned int _connectionIndex,
                            const std::string &_data)
{
  transport::ConnectionPtr conn = this->dataPtr->connections[_connectionIndex];

  if (!conn || !conn->IsOpen())
    return;

  msgs::Packet packet;
  packet.ParseFromString(_data);

  if (packet.type() == "register_topic_namespace")
  {
    msgs::GzString worldNameMsg;
    worldNameMsg.ParseFromString(packet.serialized_data());

    std::list<std::string>::iterator iter;
    iter = std::find(this->dataPtr->worldNames.begin(),
                     this->dataPtr->worldNames.end(),
                     worldNameMsg.data());
    if (iter == this->dataPtr->worldNames.end())
    {
      std::lock_guard<std::recursive_mutex>
          lock(this->dataPtr->connectionMutex);
      this->dataPtr->worldNames.push_back(worldNameMsg.data());

      Connection_M::iterator iter2;
      for (iter2 = this->dataPtr->connections.begin();
          iter2 != this->dataPtr->connections.end(); ++iter2)
      {
        iter2->second->EnqueueMsg(
            msgs::Package("topic_namespace_add", worldNameMsg));
      }
    }
  }
  else if (packet.type() == "advertise")
  {
    std::lock_guard<std::recursive_mutex> lock(this->dataPtr->connectionMutex);
    msgs::Publish pub;
    pub.ParseFromString(packet.serialized_data());

    Connection_M::iterator iter2;
    for (iter2 = this->dataPtr->connections.begin();
         iter2 != this->dataPtr->connections.end(); ++iter2)
    {
      iter2->second->EnqueueMsg(msgs::Package("publisher_add", pub));
    }

    this->dataPtr->publishers.push_back(std::make_pair(pub, conn));

    this->SendSubscribers(pub.topic(),
        msgs::Package("publisher_advertise", pub));
  }
  else if (packet.type() == "unadvertise")
  {
    msgs::Publish pub;
    pub.ParseFromString(packet.serialized_data());
    this->RemovePublisher(pub);
  }
  else if (packet.type() == "unsubscribe")
  {
    msgs::Subscribe sub;
    sub.ParseFromString(packet.serialized_data());
    this->RemoveSubscriber(sub);
  }
  else if (packet.type() == "subscribe")
  {
    msgs::Subscribe sub;
    sub.ParseFromString(packet.serialized_data());

    this->dataPtr->subscribers.push_back(std::make_pair(sub, conn));

    PubList::iterator iter;

    // Find all publishers of the topic
    for (iter = this->dataPtr->publishers.begin();
        iter != this->dataPtr->publishers.end(); ++iter)
    {
      if (iter->first.topic() == sub.topic())
      {
        conn->EnqueueMsg(msgs::Package("publisher_subscribe", iter->first));
      }
    }
  }
  else if (packet.type() == "request")
  {
    msgs::Request req;
    req.ParseFromString(packet.serialized_data());

    if (req.request() == "get_publishers")
    {
      msgs::Publishers msg;
      PubList::iterator iter;
      for (iter = this->dataPtr->publishers.begin();
          iter != this->dataPtr->publishers.end(); ++iter)
      {
        msgs::Publish *pub = msg.add_publisher();
        pub->CopyFrom(iter->first);
      }
      conn->EnqueueMsg(msgs::Package("publisher_list", msg), true);
    }
    else if (req.request() == "get_topics")
    {
      std::set<std::string> topics;
      msgs::GzString_V msg;

      // Add all topics that are published
      for (PubList::iterator iter = this->dataPtr->publishers.begin();
          iter != this->dataPtr->publishers.end(); ++iter)
      {
        topics.insert(iter->first.topic());
      }

      // Add all topics that are subscribed
      for (SubList::iterator iter = this->dataPtr->subscribers.begin();
           iter != this->dataPtr->subscribers.end(); ++iter)
      {
        topics.insert(iter->first.topic());
      }

      // Construct the message of only unique names
      for (std::set<std::string>::iterator iter =
          topics.begin(); iter != topics.end(); ++iter)
      {
        msg.add_data(*iter);
      }

      // Send the topic list message
      conn->EnqueueMsg(msgs::Package("topic_list", msg), true);
    }
    else if (req.request() == "topic_info")
    {
      msgs::Publish pub = this->GetPublisher(req.data());
      msgs::TopicInfo ti;
      ti.set_msg_type(pub.msg_type());

      PubList::iterator piter;
      SubList::iterator siter;

      // Find all publishers of the topic
      for (piter = this->dataPtr->publishers.begin();
          piter != this->dataPtr->publishers.end(); ++piter)
      {
        if (piter->first.topic() == req.data())
        {
          msgs::Publish *pubPtr = ti.add_publisher();
          pubPtr->CopyFrom(piter->first);
        }
      }

      // Find all subscribers of the topic
      for (siter = this->dataPtr->subscribers.begin();
          siter != this->dataPtr->subscribers.end(); ++siter)
      {
        if (siter->first.topic() == req.data())
        {
          // If the topic info message type has not been set or the
          // topic info message type is an empty string, then set the topic
          // info message type based on a subscriber's message type.
          if (!ti.has_msg_type() || ti.msg_type().empty())
            ti.set_msg_type(siter->first.msg_type());
          msgs::Subscribe *sub = ti.add_subscriber();
          sub->CopyFrom(siter->first);
        }
      }

      conn->EnqueueMsg(msgs::Package("topic_info_response", ti));
    }
    else if (req.request() == "get_topic_namespaces")
    {
      msgs::GzString_V msg;
      std::list<std::string>::iterator iter;
      for (iter = this->dataPtr->worldNames.begin();
          iter != this->dataPtr->worldNames.end(); ++iter)
      {
        msg.add_data(*iter);
      }
      conn->EnqueueMsg(msgs::Package("get_topic_namespaces_response", msg));
    }
    else
    {
      gzerr << "Unknown request[" << req.request() << "]\n";
    }
  }
  else
    std::cerr << "Master Unknown message type[" << packet.type()
              << "] From[" << conn->GetRemotePort() << "]\n";
}

//////////////////////////////////////////////////
void Master::Run()
{
  while (!this->dataPtr->stop)
  {
    this->RunOnce();
    common::Time::MSleep(10);
  }
}

//////////////////////////////////////////////////
void Master::RunThread()
{
  this->dataPtr->runThread = new std::thread(std::bind(&Master::Run, this));
}

//////////////////////////////////////////////////
void Master::RunOnce()
{
  Connection_M::iterator iter;

  // Process the incoming message queue
  {
    std::lock_guard<std::recursive_mutex> lock(this->dataPtr->msgsMutex);
    while (!this->dataPtr->msgs.empty())
    {
      this->ProcessMessage(this->dataPtr->msgs.front().first,
                           this->dataPtr->msgs.front().second);
      this->dataPtr->msgs.pop_front();
    }
  }

  // Process all the connections
  {
    std::lock_guard<std::recursive_mutex> lock(this->dataPtr->connectionMutex);
    for (iter = this->dataPtr->connections.begin();
        iter != this->dataPtr->connections.end();)
    {
      if (iter->second && iter->second->IsOpen())
      {
        iter->second->ProcessWriteQueue();
        ++iter;
      }
      else
      {
        this->RemoveConnection(iter++);
      }
    }
  }
}

/////////////////////////////////////////////////
void Master::RemoveConnection(Connection_M::iterator _connIter)
{
  std::list< std::pair<unsigned int, std::string> >::iterator msgIter;

  if (_connIter == this->dataPtr->connections.end() || !_connIter->second)
    return;

  // Remove all messages for this connection
  {
    std::lock_guard<std::recursive_mutex> lock(this->dataPtr->msgsMutex);
    msgIter = this->dataPtr->msgs.begin();
    while (msgIter != this->dataPtr->msgs.end())
    {
      if ((*msgIter).first == _connIter->first)
        this->dataPtr->msgs.erase(msgIter++);
      else
        ++msgIter;
    }
  }

  // Remove all publishers for this connection
  bool done = false;
  while (!done)
  {
    done = true;
    PubList::iterator pubIter = this->dataPtr->publishers.begin();
    while (pubIter != this->dataPtr->publishers.end())
    {
      if ((*pubIter).second->GetId() == _connIter->second->GetId())
      {
        this->RemovePublisher((*pubIter).first);
        done = false;
        break;
      }
      else
        ++pubIter;
    }
  }


  done = false;
  while (!done)
  {
    done = true;

    // Remove all subscribers for this connection
    SubList::iterator subIter = this->dataPtr->subscribers.begin();
    while (subIter != this->dataPtr->subscribers.end())
    {
      if ((*subIter).second->GetId() == _connIter->second->GetId())
      {
        this->RemoveSubscriber((*subIter).first);
        done = false;
        break;
      }
      else
        ++subIter;
    }
  }

  this->dataPtr->connections.erase(_connIter);
}

/////////////////////////////////////////////////
void Master::RemovePublisher(const msgs::Publish _pub)
{
  {
    std::lock_guard<std::recursive_mutex> lock(this->dataPtr->connectionMutex);
    Connection_M::iterator iter2;
    for (iter2 = this->dataPtr->connections.begin();
        iter2 != this->dataPtr->connections.end(); ++iter2)
    {
      iter2->second->EnqueueMsg(msgs::Package("publisher_del", _pub));
    }
  }

  this->SendSubscribers(_pub.topic(), msgs::Package("unadvertise", _pub));

  PubList::iterator pubIter = this->dataPtr->publishers.begin();
  while (pubIter != this->dataPtr->publishers.end())
  {
    if (pubIter->first.topic() == _pub.topic() &&
        pubIter->first.host() == _pub.host() &&
        pubIter->first.port() == _pub.port())
    {
      pubIter = this->dataPtr->publishers.erase(pubIter);
    }
    else
      ++pubIter;
  }
}

/////////////////////////////////////////////////
void Master::RemoveSubscriber(const msgs::Subscribe _sub)
{
  // Find all publishers of the topic, and remove the subscriptions
  for (PubList::iterator iter = this->dataPtr->publishers.begin();
      iter != this->dataPtr->publishers.end(); ++iter)
  {
    if (iter->first.topic() == _sub.topic())
    {
      iter->second->EnqueueMsg(msgs::Package("unsubscribe", _sub));
    }
  }

  // Remove the subscribers from our list
  SubList::iterator subiter = this->dataPtr->subscribers.begin();
  while (subiter != this->dataPtr->subscribers.end())
  {
    if (subiter->first.topic() == _sub.topic() &&
        subiter->first.host() == _sub.host() &&
        subiter->first.port() == _sub.port())
    {
      subiter = this->dataPtr->subscribers.erase(subiter);
    }
    else
      ++subiter;
  }
}

//////////////////////////////////////////////////
void Master::Stop()
{
  this->dataPtr->stop = true;

  if (this->dataPtr->runThread)
  {
    this->dataPtr->runThread->join();
    delete this->dataPtr->runThread;
    this->dataPtr->runThread = NULL;
  }
}

//////////////////////////////////////////////////
void Master::Fini()
{
  this->Stop();

  if (this->dataPtr->connection)
    this->dataPtr->connection->Shutdown();
  this->dataPtr->connection.reset();

  delete this->dataPtr->runThread;
  this->dataPtr->runThread = NULL;

  this->dataPtr->msgs.clear();
  this->dataPtr->worldNames.clear();
  this->dataPtr->connections.clear();
  this->dataPtr->subscribers.clear();
  this->dataPtr->publishers.clear();
}

//////////////////////////////////////////////////
msgs::Publish Master::GetPublisher(const std::string &_topic)
{
  msgs::Publish msg;

  PubList::iterator iter;

  // Find all publishers of the topic
  for (iter = this->dataPtr->publishers.begin();
       iter != this->dataPtr->publishers.end(); ++iter)
  {
    if (iter->first.topic() == _topic)
    {
      msg = iter->first;
      break;
    }
  }

  return msg;
}

//////////////////////////////////////////////////
transport::ConnectionPtr Master::FindConnection(const std::string &_host,
                                                uint16_t _port)
{
  transport::ConnectionPtr conn;
  Connection_M::iterator iter;

  {
    std::lock_guard<std::recursive_mutex> lock(this->dataPtr->connectionMutex);
    for (iter = this->dataPtr->connections.begin();
        iter != this->dataPtr->connections.end(); ++iter)
    {
      if (iter->second->GetRemoteAddress() == _host &&
          iter->second->GetRemotePort() == _port)
      {
        conn = iter->second;
        break;
      }
    }
  }

  return conn;
}
