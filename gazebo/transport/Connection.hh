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
#ifndef _CONNECTION_HH_
#define _CONNECTION_HH_

#include <tbb/task.h>
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
#include <utility>

#include "gazebo/common/Event.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/util/system.hh"

#define HEADER_LENGTH 8

namespace gazebo
{
  namespace transport
  {
    extern bool is_stopped();

    class IOManager;
    class Connection;
    typedef boost::shared_ptr<Connection> ConnectionPtr;

    /// \cond
    /// \brief A task instance that is created when data is read from
    /// a socket and used by TBB
    class GAZEBO_VISIBLE ConnectionReadTask : public tbb::task
    {
      /// \brief Constructor
      /// \param[_in] _func Boost function pointer, which is the function
      /// that receives the data.
      /// \param[in] _data Data to send to the boost function pointer.
      public: ConnectionReadTask(
                  boost::function<void (const std::string &)> _func,
                  const std::string &_data)
              {
                this->func = _func;
                this->data = _data;
              }

      /// \bried Overridden function from tbb::task that exectues the data
      /// callback.
      public: tbb::task *execute()
              {
                this->func(this->data);
                return NULL;
              }

      /// \brief The boost function pointer
      private: boost::function<void (const std::string &)> func;

      /// \brief The data to send to the boost function pointer
      private: std::string data;
    };
    /// \endcond

    /// \addtogroup gazebo_transport Transport
    /// \{
    ///
    /// \remarks
    ///  Environment Variables:
    ///   - GAZEBO_IP_WHITE_LIST: Comma separated list of valid IPs. Leave
    /// this empty to accept connections from all addresses.
    ///   - GAZEBO_IP: IP address to export. This will override the default
    /// IP lookup.
    ///   - GAZEBO_HOSTNAME: Hostame to export. Setting this will override
    /// both GAZEBO_IP and the default IP lookup.
    ///
    /// \class Connection Connection.hh transport/transport.hh
    /// \brief Single TCP/IP connection manager
    class GAZEBO_VISIBLE Connection :
      public boost::enable_shared_from_this<Connection>
    {
      /// \brief Constructor
      public: Connection();

      /// \brief Destructor
      public: virtual ~Connection();

      /// \brief Connect to a remote host
      /// \param[in] _host The host to connect to
      /// \param[in] _port The port to connect to
      /// \return true if connection succeeded, false otherwise
      public: bool Connect(const std::string &_host, unsigned int _port);

      /// \brief The signature of a connection accept callback
      typedef boost::function<void(const ConnectionPtr&)> AcceptCallback;

      /// \brief Start a server that listens on a port
      /// \param[in] _port The port to listen on
      /// \param[in] _acceptCB The callback to invoke when a new connection has
      /// been accepted
      public: void Listen(unsigned int _port, const AcceptCallback &_acceptCB);

      /// \brief The signature of a connection read callback
      typedef boost::function<void(const std::string &_data)> ReadCallback;

      /// \brief Start a thread that reads from the connection and passes
      ///        new message to the ReadCallback
      /// \param[in] _cb The callback to invoke when a new message is received
      public: void StartRead(const ReadCallback &_cb);

      /// \brief Stop the read loop
      public: void StopRead();

      /// \brief Shutdown the socket
      public: void Shutdown();

      /// \brief Is the connection open?
      /// \return true if the connection is open; false otherwise
      public: bool IsOpen() const;

      /// \brief Close a connection
      private: void Close();

      /// \brief Cancel all async operations on an open socket
      public: void Cancel();

      /// \brief Read data from the socket
      /// \param[out] _data Destination for data that is read
      /// \return true if data was successfully read, false otherwise
      public: bool Read(std::string &_data);

      /// \brief Write data to the socket
      /// \param[in] _buffer Data to write
      /// \param[in] _force If true, block until the data has been written
      /// to the socket, otherwise just enqueue the data for asynchronous write
      /// \param[in] _cb If non-null, callback to be invoked after
      /// transmission is complete.
      /// \param[in] _id ID associated with the message data.
      public: void EnqueueMsg(const std::string &_buffer,
                  boost::function<void(uint32_t)> _cb, uint32_t _id,
                  bool _force = false);

      /// \brief Write data to the socket
      /// \param[in] _buffer Data to write
      /// \param[in] _force If true, block until the data has been written
      /// to the socket, otherwise just enqueue the data for asynchronous write
      public: void EnqueueMsg(const std::string &_buffer, bool _force = false);

      /// \brief Get the local URI
      /// \return The local URI
      public: std::string GetLocalURI() const;

      /// \brief Get the remote URI
      /// \return The remote URI
      public: std::string GetRemoteURI() const;

      /// \brief Get the local address of this connection
      /// \return The local address
      public: std::string GetLocalAddress() const;

      /// \brief Get the port of this connection
      /// \return The local port
      public: unsigned int GetLocalPort() const;

      /// \brief Get the remote address
      /// \return The remote address
      public: std::string GetRemoteAddress() const;

      /// \brief Get the remote port number
      /// \return The remote port
      public: unsigned int GetRemotePort() const;

      /// \brief Get the remote hostname
      /// \return The remote hostname
      public: std::string GetRemoteHostname() const;

      /// \brief Get the local hostname
      /// \return The local hostname
      public: static std::string GetLocalHostname();

      /// \brief Peform an asyncronous read
      /// param[in] _handler Callback to invoke on received data
      public: template<typename Handler>
              void AsyncRead(Handler _handler)
              {
                if (!this->IsOpen())
                {
                  gzerr << "AsyncRead on a closed socket\n";
                  return;
                }

                void (Connection::*f)(const boost::system::error_code &,
                    boost::tuple<Handler>) = &Connection::OnReadHeader<Handler>;

                this->inboundHeader.resize(HEADER_LENGTH);
                boost::asio::async_read(*this->socket,
                    boost::asio::buffer(this->inboundHeader),
                    boost::bind(f, this,
                                boost::asio::placeholders::error,
                                boost::make_tuple(_handler)));
              }

      /// \brief Handle a completed read of a message header.
      ///
      /// The handler is passed using a tuple since boost::bind seems to
      /// have trouble binding a function object created using boost::bind
      /// as a parameter
      /// \param[in] _e Error code, if any, associated with the read
      /// \param[in] _handler Callback to invoke on received data
      private: template<typename Handler>
               void OnReadHeader(const boost::system::error_code &_e,
                                 boost::tuple<Handler> _handler)
              {
                if (_e)
                {
                  if (_e.message() == "End of file")
                    this->isOpen = false;
                }
                else
                {
                  std::size_t inboundData_size = 0;
                  std::string header(&this->inboundHeader[0],
                                      this->inboundHeader.size());
                  this->inboundHeader.clear();

                  inboundData_size = this->ParseHeader(header);

                 if (inboundData_size > 0)
                  {
                    // Start the asynchronous call to receive data
                    this->inboundData.resize(inboundData_size);

                    void (Connection::*f)(const boost::system::error_code &e,
                        boost::tuple<Handler>) =
                      &Connection::OnReadData<Handler>;

                    boost::asio::async_read(*this->socket,
                        boost::asio::buffer(this->inboundData),
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
                    // this->inboundHeader.resize(HEADER_LENGTH);

                    // void (Connection::*f)(const boost::system::error_code &,
                    // boost::tuple<Handler>) =
                    // &Connection::OnReadHeader<Handler>;

                    // boost::asio::async_read(*this->socket,
                    //    boost::asio::buffer(this->inboundHeader),
                    //    boost::bind(f, this,
                    //      boost::asio::placeholders::error, _handler));
                  }
                }
              }

      /// \brief Handle a completed read of a message body.
      ///
      /// The handler is passed using a tuple since boost::bind seems to
      /// have trouble binding a function object created using boost::bind
      /// as a parameter
      /// \param[in] _e Error code, if any, associated with the read
      /// \param[in] _handler Callback to invoke on received data
      private: template<typename Handler>
               void OnReadData(const boost::system::error_code &_e,
                              boost::tuple<Handler> _handler)
              {
                if (_e)
                {
                  if (_e.message() == "End of file")
                    this->isOpen = false;
                }

                // Inform caller that data has been received
                std::string data(&this->inboundData[0],
                                  this->inboundData.size());
                this->inboundData.clear();

                if (data.empty())
                  gzerr << "OnReadData got empty data!!!\n";

                if (!_e && !transport::is_stopped())
                {
                  ConnectionReadTask *task = new(tbb::task::allocate_root())
                        ConnectionReadTask(boost::get<0>(_handler), data);
                  tbb::task::enqueue(*task);

                  // Non-tbb version:
                  // boost::get<0>(_handler)(data);
                }
              }

      /// \brief Register a function to be called when the connection is shut
      /// down \param[in] _subscriber Function to be called \return Handle
      /// that can be used to unregister the function
      public: event::ConnectionPtr ConnectToShutdown(boost::function<void()>
                 _subscriber)
              { return this->shutdown.Connect(_subscriber); }

      /// \brief Unregister a function to be called when the connection is
      /// shut down \param[in] _subscriber Handle previously returned by
      /// ConnectToShutdown()
      public: void DisconnectShutdown(event::ConnectionPtr _subscriber)
              {this->shutdown.Disconnect(_subscriber);}

      /// \brief Handle on-write callbacks
      public: void ProcessWriteQueue(bool _blocking = false);

      /// \brief Get the ID of the connection.
      /// \return The connection's unique ID.
      public: unsigned int GetId() const;

      /// \brief Return true if the _ip is a valid.
      /// \param[in] _ip Dotted quad to validate.
      /// \return True if the _ip is a valid.
      public: static bool ValidateIP(const std::string &_ip);

      /// \brief Get the IP white list, from GAZEBO_IP_WHITE_LIST
      /// environment variable.
      /// \return GAZEBO_IP_WHITE_LIST
      public: std::string GetIPWhiteList() const;

      /// \brief Callback when a write has occurred.
      /// \param[in] _e Error code
      /// \param[in] _b Buffer of the data that was written.
      private: void OnWrite(const boost::system::error_code &_e);

      /// \brief Handle new connections, if this is a server
      /// \param[in] _e Error code for accept method
      private: void OnAccept(const boost::system::error_code &_e);

      /// \brief Parse a header to get the size of a packet
      /// \param[in] _header Header as a string
      private: std::size_t ParseHeader(const std::string &_header);

      /// \brief the read thread
      private: void ReadLoop(const ReadCallback &_cb);

      /// \brief Get the local endpoint
      /// \return The endpoint
      private: static boost::asio::ip::tcp::endpoint GetLocalEndpoint();

      /// \brief Get the remote endpoint
      /// \return The endpoint
      private: boost::asio::ip::tcp::endpoint GetRemoteEndpoint() const;

      /// \brief Gets hostname
      /// \param[in] _ep The end point to get the hostename of
      private: static std::string GetHostname(
                   boost::asio::ip::tcp::endpoint _ep);

      /// \brief Callback method when connected
      /// \param[in] _error Error code thrown during connection
      /// \param[in] _endPointIter Pointer to resolver iterator
      private: void OnConnect(const boost::system::error_code &_error,
                  boost::asio::ip::tcp::resolver::iterator _endPointIter);

      /// \brief Socket pointer
      private: boost::asio::ip::tcp::socket *socket;

      /// \brief Accepts new connections.
      private: boost::asio::ip::tcp::acceptor *acceptor;

      /// \brief Outgoing data queue
      private: std::deque<std::string> writeQueue;

      /// \brief List of callbacks, paired with writeQueue. The callbacks
      /// are used to notify a publisher when a message is successfully sent.
      private: std::deque<
               std::pair<boost::function<void(uint32_t)>, uint32_t> > callbacks;

      /// \brief Mutex to protect new connections.
      private: boost::mutex connectMutex;

      /// \brief Mutex to protect write.
      private: boost::recursive_mutex writeMutex;

      /// \brief Mutex to protect reads.
      private: boost::recursive_mutex readMutex;

      /// \brief Mutex to protect socket close.
      private: mutable boost::mutex socketMutex;

      /// \brief Condition used for synchronization
      private: boost::condition_variable connectCondition;

      /// \brief Called when a new connection is received
      private: AcceptCallback acceptCB;

      /// \brief Header data from a new message.
      private: std::vector<char> inboundHeader;

      /// \brief Content data from a new message.
      private: std::vector<char> inboundData;

      /// \brief Set to true to stop reading on the connection.
      private: bool readQuit;

      /// \brief Integer id of the connection.
      private: unsigned int id;

      /// \brief ID counter, used to create unique ids
      private: static unsigned int idCounter;

      /// \brief Created when a new connection is accepted.
      private: ConnectionPtr acceptConn;

      /// \brief Shutdown event
      private: event::EventT<void()> shutdown;

      /// \brief Pointer to the IO manager
      private: static IOManager *iomanager;

      /// \brief Number of writes that are being processed.
      private: unsigned int writeCount;

      /// \brief Local URI string
      private: std::string localURI;

      /// \brief Local address string
      private: std::string localAddress;

      /// \brief Remote URI string
      private: std::string remoteURI;

      /// \brief Remote address string
      private: std::string remoteAddress;

      /// \brief True if the connection has an error
      private: bool connectError;

      /// \brief Comma separated list of valid IP addresses.
      private: std::string ipWhiteList;

      /// \brief Buffer for header information.
      private: char *headerBuffer;

      /// \brief Used to prevent too many log messages.
      private: bool dropMsgLogged;

      /// \brief Index into the callbacks buffer that marks the last
      /// async_write.
      private: unsigned int callbackIndex;

      /// \brief True if the connection is open.
      private: bool isOpen;
    };
    /// \}
  }
}
#endif
