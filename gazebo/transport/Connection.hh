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
#ifndef CONNECTION_HH
#define CONNECTION_HH

#include <google/protobuf/message.h>

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/thread.hpp>
#include <boost/tuple/tuple.hpp>

#include <string>
#include <vector>
#include <iostream>
#include <iomanip>
#include <deque>


#include "common/Event.hh"
#include "common/Console.hh"
#include "common/Exception.hh"

#define HEADER_LENGTH 8

namespace gazebo
{
  namespace transport
  {
    extern bool is_stopped();

    class IOManager;
    class Connection;
    typedef boost::shared_ptr<Connection> ConnectionPtr;

    /// \addtogroup gazebo_transport
    /// \{

    /// \brief TCP/IP Connection
    class Connection : public boost::enable_shared_from_this<Connection>
    {
      /// \brief Constructor
      public: Connection();

      /// \brief Destructor
      public: virtual ~Connection();

      /// \brief Connect to a remote host
      public: bool Connect(const std::string &host, unsigned int port);

      typedef boost::function<void(const ConnectionPtr&)> AcceptCallback;

      /// \brief Start a server that listens on a port
      public: void Listen(unsigned int port, const AcceptCallback &accept_cb);

      typedef boost::function<void(const std::string &data)> ReadCallback;
      /// \brief Start a thread that reads from the connection, and passes
      ///        new message to the ReadCallback
      public: void StartRead(const ReadCallback &cb);

      /// \brief Stop the read loop
      public: void StopRead();

      /// \brief Shutdown the socket
      public: void Shutdown();

      /// \brief Return true if the connection is open
      public: bool IsOpen() const;

      /// \brief Close a connection
      private: void Close();

      /// \brief Cancel all async operations on an open socket
      public: void Cancel();

      /// \brief Read data from the socket
      public: bool Read(std::string &data);

      /// \brief Write data to the socket
      public: void EnqueueMsg(const std::string &_buffer, bool _force = false);

      /// \brief Get the local URI
      public: std::string GetLocalURI() const;

      /// \brief Get the remote URI
      public: std::string GetRemoteURI() const;

      /// \brief Get the address of this connection
      public: std::string GetLocalAddress() const;

      /// \brief Get the port of this connection
      public: unsigned int GetLocalPort() const;

      /// \brief Get the remote address
      public: std::string GetRemoteAddress() const;

      /// \brief Get the remote port number
      public: unsigned int GetRemotePort() const;

      /// \brief Get the remote hostname
      public: std::string GetRemoteHostname() const;

      /// \brief Get the local hostname
      public: std::string GetLocalHostname() const;

      /// \brief Peform and asyncronous read
      public: template<typename Handler>
              void AsyncRead(Handler handler)
              {
                if (!this->IsOpen())
                {
                  gzerr << "AsyncRead on a closed socket\n";
                  return;
                }

                void (Connection::*f)(const boost::system::error_code &,
                    boost::tuple<Handler>) = &Connection::OnReadHeader<Handler>;

                this->inbound_header.resize(HEADER_LENGTH);
                boost::asio::async_read(*this->socket,
                    boost::asio::buffer(this->inbound_header),
                    boost::bind(f, this,
                                boost::asio::placeholders::error,
                                boost::make_tuple(handler)));
              }

      // Handle a completed read of a message header. The handler is passed
      // using a tuple since boost::bind seems to have trouble binding
      // a function object created using boost::bind as a parameter
      private: template<typename Handler>
               void OnReadHeader(const boost::system::error_code &_e,
                                 boost::tuple<Handler> _handler)
              {
                if (_e)
                {
                  if (_e.message() != "End of File")
                  {
                    this->Close();
                    // This will occur when the other side closes the
                    // connection
                  }
                }
                else
                {
                  std::size_t inbound_data_size = 0;
                  std::string header(&this->inbound_header[0],
                                      this->inbound_header.size());
                  this->inbound_header.clear();

                  inbound_data_size = this->ParseHeader(header);

                 if (inbound_data_size > 0)
                  {
                    // Start the asynchronous call to receive data
                    this->inbound_data.resize(inbound_data_size);

                    void (Connection::*f)(const boost::system::error_code &e,
                        boost::tuple<Handler>) =
                      &Connection::OnReadData<Handler>;

                    boost::asio::async_read(*this->socket,
                        boost::asio::buffer(this->inbound_data),
                        boost::bind(f, this,
                                    boost::asio::placeholders::error,
                                    _handler));
                  }
                  else
                  {
                    gzerr << "Header is empty\n";
                    boost::get<0>(_handler)("");
                    // This code tries to read the header again. We should
                    // never get here.
                    // this->inbound_header.resize(HEADER_LENGTH);

                    // void (Connection::*f)(const boost::system::error_code &,
                    // boost::tuple<Handler>) =
                    // &Connection::OnReadHeader<Handler>;

                    // boost::asio::async_read(*this->socket,
                    //    boost::asio::buffer(this->inbound_header),
                    //    boost::bind(f, this,
                    //      boost::asio::placeholders::error, _handler));
                  }
                }
              }

     private: template<typename Handler>
              void OnReadData(const boost::system::error_code &e,
                              boost::tuple<Handler> handler)
              {
                if (e)
                  gzerr << "Error Reading data!\n";

                // Inform caller that data has been received
                std::string data(&this->inbound_data[0],
                                  this->inbound_data.size());
                this->inbound_data.clear();

                if (data.empty())
                  gzerr << "OnReadData got empty data!!!\n";

                if (!e && !transport::is_stopped())
                {
                  boost::get<0>(handler)(data);
                }
              }

     public: event::ConnectionPtr ConnectToShutdown(boost::function<void()>
                 subscriber_)
             { return this->shutdown.Connect(subscriber_); }

     public: void DisconnectShutdown(event::ConnectionPtr subscriber_)
             {this->shutdown.Disconnect(subscriber_);}

     /// \brief Handle on write callbacks
     public: void ProcessWriteQueue();

     private: void OnWrite(const boost::system::error_code &e,
                  boost::asio::streambuf *_b);

     /// \brief Handle new connections, if this is a server
     private: void OnAccept(const boost::system::error_code &e);

     /// \brief Parse a header to get the size of a packet
     private: std::size_t ParseHeader(const std::string &header);

     /// \brief the read thread
     private: void ReadLoop(const ReadCallback &cb);

     /// \brief Get the local endpoint
     private: boost::asio::ip::tcp::endpoint GetLocalEndpoint() const;

     /// \brief Get the remote endpoint
     private: boost::asio::ip::tcp::endpoint GetRemoteEndpoint() const;

     private: static std::string GetHostname(boost::asio::ip::tcp::endpoint ep);

     private: void OnConnect(const boost::system::error_code &_error,
                  boost::asio::ip::tcp::resolver::iterator _endPointIter);

      private: boost::asio::ip::tcp::socket *socket;
      private: boost::asio::ip::tcp::acceptor *acceptor;

      private: std::deque<std::string> writeQueue;
      private: std::deque<unsigned int> writeCounts;
      private: boost::mutex *connectMutex;
      private: boost::recursive_mutex *writeMutex;
      private: boost::recursive_mutex *readMutex;

      private: boost::condition_variable connectCondition;

      // Called when a new connection is received
      private: AcceptCallback acceptCB;

      private: std::vector<char> inbound_header;
      private: std::vector<char> inbound_data;

      private: boost::thread *readThread;
      private: bool readQuit;

      public: unsigned int id;
      private: static unsigned int idCounter;
      private: ConnectionPtr acceptConn;

      private: event::EventT<void()> shutdown;
      private: static IOManager *iomanager;

      public: unsigned int writeCount;

      private: std::string localURI;
      private: std::string localAddress;
      private: std::string remoteURI;
      private: std::string remoteAddress;

      private: bool connectError;
    };
    /// \}
  }
}

#endif


