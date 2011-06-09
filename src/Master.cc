/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
#include <google/protobuf/descriptor.h>
#include "transport/IOManager.hh"

#include "Master.hh"

#include "gazebo_config.h"

using namespace gazebo;

Master::Master()
  : connection( new transport::Connection() )
{
  this->quit = false;
}

Master::~Master()
{
  this->publishers.clear();
  this->subscribers.clear();

  for (std::deque<transport::ConnectionPtr>::iterator iter = this->connections.begin(); iter != this->connections.end(); iter++)
  {
    (*iter)->Shutdown();
    (*iter).reset();
  }

  this->connections.clear();
  this->connection->Shutdown();
  this->connection.reset();
}

void Master::Init(unsigned short port)
{
  try
  {
    this->connection->Listen(port, boost::bind(&Master::OnAccept, this, _1));
  }
  catch (std::exception &e)
  {
    gzthrow( "Unable to start server[" << e.what() << "]\n");
  }
}

void Master::OnAccept(const transport::ConnectionPtr &new_connection)
{
  msgs::String msg;
  msg.set_data(std::string("gazebo ") + GAZEBO_VERSION);

  new_connection->EnqueueMsg( common::Message::Package("init", msg), true );

  this->connections.push_back(new_connection);
  new_connection->StartRead( boost::bind(&Master::OnRead, this, this->connections.size()-1, _1));
}

void Master::OnRead(const unsigned int connectionIndex, 
                    const std::string &data)
{
  transport::ConnectionPtr conn = this->connections[connectionIndex];
  msgs::Packet packet;
  packet.ParseFromString(data);

  if (packet.type() == "advertise")
  {
    msgs::Publish pub;
    pub.ParseFromString( packet.serialized_data() );

    this->publishers.push_back( std::make_pair(pub, conn) );

    SubList::iterator iter;

    // Find all subscribers of the topic
    for (iter = this->subscribers.begin(); 
         iter != this->subscribers.end(); iter++)
    {
      if (iter->first.topic() == pub.topic())
      {
        iter->second->EnqueueMsg(common::Message::Package("publisher_update", pub));
      }
    }
  }
  else if (packet.type() == "unadvertise")
  {
    msgs::Publish pub;
    pub.ParseFromString( packet.serialized_data() );

    SubList::iterator iter;
    // Find all subscribers of the topic
    for (iter = this->subscribers.begin(); 
         iter != this->subscribers.end(); iter++)
    {
      if (iter->first.topic() == pub.topic())
      {
        iter->second->EnqueueMsg(common::Message::Package("unadvertise", pub));
      }
    }

  }
  else if (packet.type() == "unsubscribe")
  {
    msgs::Subscribe sub;
    sub.ParseFromString( packet.serialized_data() );
    SubList::iterator subiter = this->subscribers.begin();

    // Find all publishers of the topic, and remove the subscriptions
    for (PubList::iterator iter = this->publishers.begin(); 
         iter != this->publishers.end(); iter++)
    {
      if (iter->first.topic() == sub.topic())
      {
        iter->second->EnqueueMsg(common::Message::Package("unsubscribe", sub));
      }
    }

    // Remove the subscribers from our list
    while (subiter != this->subscribers.end())
    {
      if (subiter->first.topic() == sub.topic() && 
          subiter->first.host() == sub.host() &&
          subiter->first.port() == sub.port())
      {
        this->subscribers.erase(subiter++);
      }
      else
        subiter++;
    }
  }
  else if (packet.type() == "subscribe")
  {
    msgs::Subscribe sub;
    sub.ParseFromString( packet.serialized_data() );

    this->subscribers.push_back( std::make_pair(sub, conn) );

    PubList::iterator iter;

    // Find all publishers of the topic
    for (iter = this->publishers.begin(); 
         iter != this->publishers.end(); iter++)
    {
      if (iter->first.topic() == sub.topic())
      {
        conn->EnqueueMsg(common::Message::Package("publisher_update", iter->first), true);
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
      for (iter = this->publishers.begin(); 
           iter != this->publishers.end(); iter++)
      {
        msgs::Publish *pub = msg.add_publisher();
        pub->CopyFrom( iter->first );
      }
      conn->EnqueueMsg( common::Message::Package("publisher_list", msg), true );
    }
    else if (req.request() == "topic_info")
    {
      msgs::Publish pub = this->GetPublisher( req.str_data() );
      msgs::TopicInfo ti;
      ti.set_msg_type( pub.msg_type() );

      PubList::iterator piter;
      SubList::iterator siter;

      // Find all publishers of the topic
      for (piter = this->publishers.begin(); 
           piter != this->publishers.end(); piter++)
      {
        if ( piter->first.topic() == req.str_data())
        {
          msgs::Publish *pubPtr = ti.add_publisher();
          pubPtr->CopyFrom( piter->first );
        }
      }

      // Find all subscribers of the topic
      for (siter = this->subscribers.begin(); 
           siter != this->subscribers.end(); siter++)
      {
        if ( siter->first.topic() == req.str_data())
        {
          msgs::Subscribe *sub = ti.add_subscriber();
          sub->CopyFrom( siter->first );
        }
      }

      conn->EnqueueMsg( common::Message::Package("topic_info_response", ti), true );
    }
  }
  else
    std::cerr << "Master Unknown message type\n";
}

void Master::Run()
{
  std::deque<transport::ConnectionPtr>::iterator iter;
  while (!this->quit)
  {
    for (iter = this->connections.begin(); 
         iter != this->connections.end(); iter++)
    {
      if ((*iter)->IsOpen())
        (*iter)->ProcessWriteQueue();
      else
        std::cout << "Master connection has closed\n";
    }
    usleep(100000);
  }
}

void Master::Quit()
{
  this->quit = true;
}

msgs::Publish Master::GetPublisher( const std::string &topic )
{
  msgs::Publish msg;

  PubList::iterator iter;

  // Find all publishers of the topic
  for (iter = this->publishers.begin(); 
       iter != this->publishers.end(); iter++)
  {
    if (iter->first.topic() == topic)
    {
      msg = iter->first;
      break;
    }
  }

  return msg;
}

transport::ConnectionPtr Master::FindConnection(const std::string &host, unsigned short port)
{
  transport::ConnectionPtr conn;
  std::deque<transport::ConnectionPtr>::iterator iter;


  for (iter = this->connections.begin(); 
       iter != this->connections.end(); iter++)
  {
    if ((*iter)->GetRemoteAddress() == host && (*iter)->GetRemotePort() == port)
    {
      conn = (*iter);
      break;
    }
  }

  return conn;
}
