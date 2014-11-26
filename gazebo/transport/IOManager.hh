/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#ifndef _IOMANAGER_HH_
#define _IOMANAGER_HH_

#include <boost/thread/thread.hpp>
#include <boost/asio.hpp>
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace transport
  {
    /// \addtogroup gazebo_transport
    /// \{

    /// \class IOManager IOManager.hh transport/transport.hh
    /// \brief Manages boost::asio IO
    class GZ_TRANSPORT_VISIBLE IOManager
    {
      /// \brief Constructor
      public: IOManager();
      /// \brief Destructor
      public: ~IOManager();

      /// \brief Get handle to boost::asio IO service
      /// \return Handle to boost::asio IO service
      public: boost::asio::io_service &GetIO();

      /// \brief Increment the event count by 1
      public: void IncCount();

      /// \brief Decrement the event count by 1
      public: void DecCount();

      /// \brief Get the event count
      /// \return The event count
      public: unsigned int GetCount() const;

      /// \brief Stop the IO service
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


