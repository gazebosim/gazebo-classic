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
#ifndef CONNECTION_HH
#define CONNECTION_HH

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/thread.hpp>
#include <boost/tuple/tuple.hpp>
#include <iostream>
#include <iomanip>

#include <google/protobuf/message.h>

#include "common/Console.hh"
#include "common/Exception.hh"

#define HEADER_LENGTH 8

namespace gazebo
{
  namespace transport
  {
    class IOManager;
    class Connection;
    typedef boost::shared_ptr<Connection> ConnectionPtr;

    class Connection
    {
      /// \brief Constructor
      public: Connection();

      /// \brief Destructor
      public: virtual ~Connection();

      /// \brief Connect to a remote host
      public: void Connect(const std::string &host,  unsigned short port);

      typedef boost::function<void(const ConnectionPtr&)> AcceptCallback;

      /// \brief Start a server that listens on a port
      public: void Listen(unsigned short port, const AcceptCallback &accept_cb);

      typedef boost::function<void(const std::string &data)> ReadCallback;
      /// \brief Start a thread that reads from the connection, and passes
      ///        new message to the ReadCallback
      public: void StartRead(const ReadCallback &cb);
             
      /// \brief Stop the read loop 
      public: void StopRead();

      /// \brief Cancel all async operations on an open socket
      public: void Cancel();

      /// \brief Read data from the socket
      public: void Read(std::string &data);

      /// \brief Write data to the socket
      public: void Write(const std::string &buffer);

      /// \brief Get the local URI
      public: std::string GetLocalURI() const;
              
      /// \brief Get the remote URI
      public: std::string GetRemoteURI() const;

      /// \brief Get the address of this connection
      public: std::string GetLocalAddress() const;

      /// \brief Get the port of this connection
      public: unsigned short GetLocalPort() const;

      /// \brief Get the remote address
      public: std::string GetRemoteAddress() const;

      /// \brief Get the remote port number
      public: unsigned short GetRemotePort() const;

      /// \brief Get the remote hostname
      public: std::string GetRemoteHostname() const;

      /// \brief Get the local hostname
      public: std::string GetLocalHostname() const;

      /// \brief Peform and asyncronous read
      public: template<typename Handler>
              void AsyncRead(Handler handler)
              {
                void (Connection::*f)(const boost::system::error_code &,
                    boost::tuple<Handler>) = &Connection::OnReadHeader<Handler>;

                boost::asio::async_read(this->socket,
                    boost::asio::buffer(this->inbound_header),
                    boost::bind(f, this, boost::asio::placeholders::error,
                                boost::make_tuple(handler)) );
              }

      // Handle a completed read of a message header. The handler is passed
      // using a tuple since boost::bind seems to have trouble binding
      // a function object created using boost::bind as a parameter
      private: template<typename Handler>
               void OnReadHeader(const boost::system::error_code &e,
                                 boost::tuple<Handler> handler)
              {
                if (e)
                {
                  std::cerr << "An error occrured reading a header[" 
                            << e.message() << "]\n";
                  // Pass the error to the handler
                  //boost::get<0>(handler)(e);
                }
                else
                {
                  std::size_t inbound_data_size = 0;
                  try
                  {
                    inbound_data_size = this->ParseHeader(this->inbound_header);
                  }
                  catch (gazebo::common::Exception &e)
                  {
                    gzerr << "Error[" << e << "]\n";
                  }

                  // Start the asynchronous call to receive data
                  this->inbound_data.resize(inbound_data_size);

                  void (Connection::*f)(const boost::system::error_code &e,
                     boost::tuple<Handler>) = &Connection::OnReadData<Handler>;

                  boost::asio::async_read( this->socket, 
                      boost::asio::buffer(this->inbound_data), 
                      boost::bind(f, this, boost::asio::placeholders::error, 
                                  handler) );
                }
              }

     private: template<typename Handler>
              void OnReadData(const boost::system::error_code &e,
                              boost::tuple<Handler> handler)
               {
                 if (e)
                 {
                   gzerr << "Error:" << e.message() << std::endl;
                 }
                 else
                 {
                   // Inform caller that data has been received
                   std::string data(&this->inbound_data[0], 
                                    this->inbound_data.size());
                   boost::get<0>(handler)(data);
                 }
               }


      /// \brief Handle on write callbacks
     private: void OnWrite(const boost::system::error_code &e);

     /// \brief Handle new connections, if this is a server
     private: void OnAccept(const boost::system::error_code &e,
                            ConnectionPtr newConnection);

     /// \brief Parse a header to get the size of a packet
     private: std::size_t ParseHeader( const std::string &header );

     /// \brief the read thread
     private: void ReadLoop(const ReadCallback &cb);

     /// \brief Get the local endpoint
     private: boost::asio::ip::tcp::endpoint GetLocalEndpoint() const;

     /// \brief Get the remote endpoint
     private: boost::asio::ip::tcp::endpoint GetRemoteEndpoint() const;

     private: std::string GetHostname(boost::asio::ip::tcp::endpoint ep) const;

      private: boost::asio::ip::tcp::socket socket;
      private: boost::asio::ip::tcp::acceptor *acceptor;

      private: std::string outbound_header;
      private: std::string outbound_data;

      // Called when a new connection is received
      private: AcceptCallback acceptCB;

      private: char inbound_header[HEADER_LENGTH];
      private: std::vector<char> inbound_data;

      private: boost::thread *readThread;
      private: bool readQuit;
    };

  }
}

#endif
