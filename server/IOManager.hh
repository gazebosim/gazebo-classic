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
#ifndef IOMANAGER_HH
#define IOMANAGER_HH

#include <boost/thread/thread.hpp>
#include <boost/asio.hpp>

namespace gazebo
{
  class IOManager;
  typedef boost::shared_ptr<IOManager> IOManagerPtr;

  class IOManager
  {
    public: static const IOManagerPtr &Instance();

    public: void Start();
    public: void Stop();

    public: boost::asio::io_service &GetIO();

    private: IOManager();
    public: ~IOManager();

    private: boost::asio::io_service io_service;

    // Use io_service::work to keep the io_service running in thread
    private: boost::asio::io_service::work work;

    private: boost::thread thread;

    private: static IOManagerPtr self;
  };
}

#endif
