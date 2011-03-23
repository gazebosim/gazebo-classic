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
#include <boost/tuple/tuple.hpp>
#include <iostream>
#include <iomanip>

#include <google/protobuf/message.h>

#include "common/GazeboError.hh"

#define HEADER_LENGTH 8

namespace gazebo
{
  namespace transport
  {
    class Connection
    {
      public: Connection(boost::asio::io_service &io_service);
      public: virtual ~Connection();

      /// \brief Connect to a remote host
      public: void Connect(const std::string &host, const std::string &service);

      /// \brief Start a server that listens on a port
      public: void Listen(unsigned short port, const AcceptCallback &accept_cb);

      public: void StartReadThread();

      /// \brief Get the number of messages in the read buffer
      public: unsigned int GetReadBufferSize();

      /// \brief Pop one message off the read buffer, and return the
      ///        serialized data in msg
      public: void PopReadBuffer(std::string &msg);

      /// \brief Get the address of this connection
      public: std::string GetLocalAddress() const;

      /// \brief Get the port of this connection
      public: unsigned short GetLocalPort() const;

      /// \brief Get the remote address
      public: std::string GetRemoteAddress() const;

      /// \brief Get the remote port number
      public: unsigned short GetRemotePort() const;

      public: void Write(const google::protobuf::Message &msg)
              {
                std::string out;
                if (!msg.SerializeToString(&out))
                  gzthrow("Failed to serialized message");

                this->Write(out, boost::bind(&Connection::OnWrite, this,
                      boost::asio::placeholders::error));
              }


      private: template<typename Handler>
              void Write(const std::string buffer, Handler handler)
              {
                std::ostringstream header_stream;
                header_stream << std::setw(HEADER_LENGTH) 
                              << std::hex << buffer.size();

                if (!header_stream || header_stream.str().size() != HEADER_LENGTH)
                {
                  //Something went wrong, inform the caller
                  boost::system::error_code error(boost::asio::error::invalid_argument);
                  this->socket.io_service().post(boost::bind(handler,error));
                }

                this->outbound_header = header_stream.str();
                this->outbound_data = buffer;

                // Write the serialized data to the socket. We use
                // "gather-write" to send both the head and the data in
                // a single write operation
                std::vector<boost::asio::const_buffer> buffers;
                buffers.push_back(boost::asio::buffer(this->outbound_header));
                buffers.push_back(boost::asio::buffer(this->outbound_data));
                boost::asio::async_write( this->socket, buffers, handler );
              }

      public: template<typename Handler>
              void Read(Handler handler)
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
      public: template<typename Handler>
              void OnReadHeader(const boost::system::error_code &e,
                                boost::tuple<Handler> handler)
              {
                if (e)
                {
                  std::cout << "An error occrured reading a header\n";
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
                  catch (gazebo::common::GazeboError &e)
                  {
                    std::cerr << "Error[" << e << "]\n";
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

     public: template<typename Handler>
              void OnReadData(const boost::system::error_code &e,
                              boost::tuple<Handler> handler)
               {
                 if (e)
                 {
                   std::cerr << "Error:" << e.message() << std::endl;
                 }
                 else
                 {
                   // Inform caller that data has been received
                   std::string data(&this->inbound_data[0], 
                                    this->inbound_data.size());
                   boost::get<0>(handler)(data);
                 }
               }


     private: void ReadLoop();
     private: std::size_t ParseHeader( const std::string header );
     private: void OnWrite(const boost::system::error_code &e);

      private: boost::asio::ip::tcp::socket socket;
      private: std::string outbound_header;
      private: std::string outbound_data;

      private: char inbound_header[HEADER_LENGTH];
      private: std::vector<char> inbound_data;

      private: boost::thread *readThread;
      private: boost::mutex *readBufferMutex;
      private: std::list< std::string > readBuffer;
      private: bool readQuit;
    };

    typedef boost::shared_ptr<Connection> ConnectionPtr;
  }
}

#endif
