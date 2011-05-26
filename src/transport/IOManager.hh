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
#include "common/SingletonT.hh"

namespace gazebo
{
  namespace transport
  {
    class IOManager : public SingletonT<IOManager>
    {
      public: boost::asio::io_service &GetIO();

      public: void IncCount();
      public: void DecCount();
      public: unsigned int GetCount() const;

      private: IOManager();
      private: ~IOManager();

      private: boost::asio::io_service io_service;

      // Use io_service::work to keep the io_service running in thread
      private: boost::asio::io_service::work work;
      private: unsigned int count;

      private: boost::thread thread;

      //Singleton implementation
      private: friend class SingletonT<IOManager>;
    };
  }
}

#endif
