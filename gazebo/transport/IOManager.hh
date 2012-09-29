/*
 * Copyright 2011 Nate Koenig
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
  namespace transport
  {
    /// \addtogroup gazebo_transport
    /// \{

    /// \brief Managers boost::asio IO
    class IOManager
    {
      public: IOManager();
      public: ~IOManager();

      public: boost::asio::io_service &GetIO();

      public: void IncCount();
      public: void DecCount();
      public: unsigned int GetCount() const;

      public: void Stop();

      private: boost::asio::io_service *io_service;

      // Use io_service::work to keep the io_service running in thread
      private: boost::asio::io_service::work *work;
      private: unsigned int count;

      private: boost::thread *thread;
    };
    /// \}
  }
}

#endif


