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

#include "transport/IOManager.hh"

#include "Master.hh"

#include "gazebo_config.h"

using namespace gazebo;

Master::Master()
  : connection( new transport::Connection() )
{
  transport::IOManager::Instance()->Start();
  this->quit = false;
}

Master::~Master()
{
  transport::IOManager::Instance()->Stop();
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
  this->connections.push_back(new_connection);

  msgs::String msg;
  msg.set_data(std::string("gazebo ") + GAZEBO_VERSION);
  new_connection->Write( common::Message::Package("init", msg) );

  new_connection->StartRead( boost::bind(&Master::OnRead, this, new_connection, _1));
}

void Master::OnRead(const transport::ConnectionPtr &conn, 
                    const std::string &data)
{
  msgs::Packet packet;
  packet.ParseFromString(data);

  if (packet.type() == "advertise")
  {
    msgs::Publish pub;
    pub.ParseFromString( packet.serialized_data() );

    this->publishers.push_back( pub );

    std::cout << "new publish message[" << pub.DebugString();
  }
  else if (packet.type() == "subscribe")
  {
    msgs::Subscribe sub;
    sub.ParseFromString( packet.serialized_data() );

    this->subscribers.push_back( sub );

    std::list<msgs::Publish>::iterator iter;
    std::cout << "new subscribe message[" << sub.DebugString();

    // Find all publishers of the topic
    for (iter = this->publishers.begin(); 
         iter != this->publishers.end(); iter++)
    {
      if ((*iter).topic() == sub.topic())
      {
        std::cout << "Found a publisher\n";
        std::cout << (*iter).DebugString();
        conn->Write( common::Message::Package("publisher_update", *iter) );
      }
    }
  }
  else
    std::cerr << "Master Unknown message type\n";
}

void Master::Run()
{
  while (!this->quit)
  {
    usleep(1000000);
  }
}

void Master::Quit()
{
  this->quit = true;
}
