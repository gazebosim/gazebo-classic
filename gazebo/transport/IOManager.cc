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
#include <atomic>
#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>
#include <iostream>
#include "gazebo/transport/IOManager.hh"

namespace gazebo
{
namespace transport
{
/////////////////////////////////////////////////
class IOManagerPrivate
{
  /// \brief IO service.
  public: boost::asio::io_service *io_service = nullptr;

  /// \brief Use io_service::work to keep the io_service running in thread.
  public: boost::asio::io_service::work *work = nullptr;

  /// \brief Reference count of connections using this IOManager.
  public: std::atomic_int count;

  /// \brief Thread for IOManager.
  public: boost::thread *thread = nullptr;
};

/////////////////////////////////////////////////
IOManager::IOManager()
  : dataPtr(new IOManagerPrivate)
{
  this->dataPtr->io_service = new boost::asio::io_service;
  this->dataPtr->work = new boost::asio::io_service::work(
      *this->dataPtr->io_service);
  this->dataPtr->count = 0;
  this->dataPtr->thread = new boost::thread(boost::bind(
      &boost::asio::io_service::run, this->dataPtr->io_service));
}

/////////////////////////////////////////////////
IOManager::~IOManager()
{
  this->Stop();

  delete this->dataPtr->work;
  this->dataPtr->work = nullptr;

  delete this->dataPtr->io_service;
  this->dataPtr->io_service = nullptr;
}

/////////////////////////////////////////////////
void IOManager::Stop()
{
  this->dataPtr->io_service->reset();
  this->dataPtr->io_service->stop();
  if (this->dataPtr->thread)
  {
    this->dataPtr->thread->join();
    delete this->dataPtr->thread;
    this->dataPtr->thread = nullptr;
  }
}

/////////////////////////////////////////////////
boost::asio::io_service &IOManager::GetIO()
{
  return *this->dataPtr->io_service;
}

/////////////////////////////////////////////////
void IOManager::IncCount()
{
  this->dataPtr->count++;
}

/////////////////////////////////////////////////
void IOManager::DecCount()
{
  this->dataPtr->count--;
}

/////////////////////////////////////////////////
unsigned int IOManager::GetCount() const
{
  return this->dataPtr->count;
}
}
}
