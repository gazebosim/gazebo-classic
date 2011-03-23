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
#include "transport/Server.hh"

#include "Master.hh"

using namespace gazebo;

Master::Master()
{
  transport::IOManager::Instance()->Start();
  this->server = NULL;
  this->quit = false;
}

Master::~Master()
{
  if (this->server)
    delete this->server;
  transport::IOManager::Instance()->Stop();
}

void Master::HandlePublish(const boost::shared_ptr<msgs::Publish const> &msg)
{
  std::cout << "Handle Publish\n";
  this->publishers.push_back( *msg );
}

void Master::HandleSubscribe(const boost::shared_ptr<msgs::Subscribe const> &msg)
{
  std::cout << "Handle Suscribe\n";
  this->subscribers.push_back( *msg );
}

void Master::Init(unsigned short port)
{
  try
  {
    this->server = new transport::Server(port);
  }
  catch (std::exception &e)
  {
    gzthrow( "Unable to start server[" << e.what() << "]\n");
  }

  this->server->Subscribe("/gazebo/publish", &Master::HandlePublish, this);
  this->server->Subscribe("/gazebo/subscribe", &Master::HandleSubscribe, this);
}

void Master::Run()
{
  while (!this->quit)
  {
    this->server->ProcessIncoming();
    usleep(1000000);
  }
}

void Master::Quit()
{
  this->quit = true;
}
