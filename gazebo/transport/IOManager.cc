/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
#include <boost/bind.hpp>
#include <iostream>
#include "gazebo/transport/IOManager.hh"

using namespace gazebo;
using namespace transport;

/////////////////////////////////////////////////
IOManager::IOManager()
  : count(0)
{
  this->io_service = new boost::asio::io_service;
  this->work = new boost::asio::io_service::work(*this->io_service);
  this->thread = new boost::thread(boost::bind(&boost::asio::io_service::run,
                                                this->io_service));
}

/////////////////////////////////////////////////
IOManager::~IOManager()
{
  this->Stop();

  delete this->work;
  this->work = NULL;

  delete this->io_service;
  this->io_service = NULL;
}

/////////////////////////////////////////////////
void IOManager::Stop()
{
  this->io_service->reset();
  this->io_service->stop();
  if (this->thread)
  {
    this->thread->join();
    delete this->thread;
    this->thread = NULL;
  }
}

/////////////////////////////////////////////////
boost::asio::io_service &IOManager::GetIO()
{
  return *this->io_service;
}

/////////////////////////////////////////////////
void IOManager::IncCount()
{
  this->count++;
}

/////////////////////////////////////////////////
void IOManager::DecCount()
{
  this->count--;
}

/////////////////////////////////////////////////
unsigned int IOManager::GetCount() const
{
  return this->count;
}
