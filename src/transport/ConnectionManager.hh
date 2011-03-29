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
#ifndef CONNECTION_MANAGER_HH
#define CONNECTION_MANAGER_HH

#include <string>
#include <boost/shared_ptr.hpp>

#include "common/SingletonT.hh"
#include "common/Messages.hh"

#include "transport/Publisher.hh"
#include "transport/Connection.hh"

namespace gazebo
{
  namespace transport
  {
    class ConnectionManager : public SingletonT<ConnectionManager>
    {
      /// \brief Constructor
      private: ConnectionManager();

      /// \brief Destructor
      private: virtual ~ConnectionManager();

      public: void Init( const std::string &master_host, 
                         unsigned short master_port);

      public: void Subscribe( const std::string &topic, 
                              const std::string &msgType);

      public: void Advertise( const std::string &topic, 
                              const std::string &msgType);

      /// \brief Explicitly update the publisher list
      public: void GetAllPublishers( std::list<msgs::Publish> &publishers );

      /// \brief Find a connection that matches a host and port
      private: ConnectionPtr FindConnection(const std::string &host, 
                                            unsigned short port);
               
      /// \brief Connect to a remote server
      private: ConnectionPtr ConnectToRemoteHost( const std::string &host,
                                                  unsigned short port);

      private: void OnMasterRead( const std::string &data );

      private: void OnAccept(const ConnectionPtr &new_connection);

      private: void OnRead( const ConnectionPtr &new_connection,
                            const std::string &data );

      private: ConnectionPtr masterConn;
      private: ConnectionPtr serverConn;

      private: std::list<ConnectionPtr> connections;

      //Singleton implementation
      private: friend class SingletonT<ConnectionManager>;
    };
  }
}

#endif
