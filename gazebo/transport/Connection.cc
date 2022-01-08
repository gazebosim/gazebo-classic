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

#ifdef _WIN32
  // For socket(), connect(), send(), and recv().
  #include <Winsock2.h>
  #include <Ws2def.h>
  #include <Ws2ipdef.h>
  #include <Ws2tcpip.h>
  #include <iphlpapi.h>
  // Type used for raw data on this platform.
  typedef char raw_type;
  // snprintf is available since VS 2015
  #if defined(_MSC_VER) && (_MSC_VER < 1900)
    #define snprintf _snprintf
  #endif
#else
  // For data types
  #include <sys/types.h>
  // For socket(), connect(), send(), and recv()
  #include <sys/socket.h>
  // For gethostbyname()
  #include <netdb.h>
  // For inet_addr()
  #include <arpa/inet.h>
  // For close()
  #include <unistd.h>
  // For sockaddr_in
  #include <netinet/in.h>
  // Type used for raw data on this platform
  typedef void raw_type;
  #include <ifaddrs.h>
#endif

#include <stdio.h>
#include <stdlib.h>

#include <boost/bind/bind.hpp>
#include <boost/function.hpp>
#include <boost/lexical_cast.hpp>

#include "gazebo/common/Console.hh"
#include "gazebo/msgs/msgs.hh"

#include "gazebo/transport/IOManager.hh"
#include "gazebo/transport/ConnectionManager.hh"
#include "gazebo/transport/Connection.hh"

using namespace gazebo;
using namespace transport;

extern void dummy_callback_fn(uint32_t);

unsigned int Connection::idCounter = 0;
IOManager *Connection::iomanager = NULL;

// Version 1.52 of boost has an address::is_unspecfied function, but
// Version 1.46.1 (installed on ubuntu) does not. So this helper function
// is stolen from adress::is_unspecified function in boost v1.52.
static bool addressIsUnspecified(const boost::asio::ip::address_v4 &_addr)
{
  return _addr.to_ulong() == 0;
}

// Version 1.52 of boost has an address::is_loopback function, but
// Version 1.46.1 (installed on ubuntu) does not. So this helper function
// is stolen from adress::is_loopback function in boost v1.52.
static bool addressIsLoopback(const boost::asio::ip::address_v4 &_addr)
{
  return (_addr.to_ulong() & 0xFF000000) == 0x7F000000;
}

//////////////////////////////////////////////////
Connection::Connection()
{
  this->isOpen = false;
  this->dropMsgLogged = false;

  if (iomanager == NULL)
    iomanager = new IOManager();

  this->socket = new boost::asio::ip::tcp::socket(iomanager->GetIO());

  iomanager->IncCount();
  this->id = idCounter++;

  this->acceptor = NULL;
  this->readQuit = false;
  this->connectError = false;
  this->writeQueue.clear();
  this->writeCount = 0;

  this->localURI = std::string("http://") + this->GetLocalHostname() + ":" +
                   boost::lexical_cast<std::string>(this->GetLocalPort());

  this->localAddress = this->GetLocalEndpoint().address().to_string();

  // Get and set the IP white list from the GAZEBO_IP_WHITE_LIST environment
  // variable.
  char *whiteListEnv = getenv("GAZEBO_IP_WHITE_LIST");
  if (whiteListEnv && !std::string(whiteListEnv).empty())
  {
    // Automatically add in the local addresses. This guarantees that
    // Gazebo will run properly on the local machine.
    this->ipWhiteList = "," + this->localAddress + ",127.0.0.1,"
      + whiteListEnv + ",";
  }
}

//////////////////////////////////////////////////
Connection::~Connection()
{
  this->Shutdown();

  if (iomanager)
  {
    iomanager->DecCount();
    if (iomanager->GetCount() == 0)
    {
      this->idCounter = 0;
      delete iomanager;
      iomanager = NULL;
    }
  }
}

//////////////////////////////////////////////////
bool Connection::Connect(const std::string &_host, unsigned int _port)
{
  boost::mutex::scoped_lock lock(this->connectMutex);

  std::string service = boost::lexical_cast<std::string>(_port);

  int httpIndex = _host.find("http://");
  std::string host = _host;
  if (httpIndex != static_cast<int>(std::string::npos))
    host = _host.substr(7, _host.size() - 7);

  // Resolve the host name into an IP address
  boost::asio::ip::tcp::resolver::iterator end;
  boost::asio::ip::tcp::resolver resolver(iomanager->GetIO());
  boost::asio::ip::tcp::resolver::query query(host, service,
      boost::asio::ip::resolver_query_base::numeric_service);
  boost::asio::ip::tcp::resolver::iterator endpointIter;

  try
  {
    endpointIter = resolver.resolve(query);

    // Find the first valid IPv4 address
    for (; endpointIter != end &&
           !(*endpointIter).endpoint().address().is_v4(); ++endpointIter)
    {
    }

    // Make sure we didn't run off the end of the list.
    if (endpointIter == end)
    {
      gzerr << "Unable to resolve uri[" << _host << ":" << _port << "]\n";
      return false;
    }
  }
  catch(...)
  {
    gzerr << "Unable to resolve uri[" << host << ":" << _port << "]\n";
    return false;
  }

  this->connectError = false;
  this->remoteURI.clear();

  // Use async connect so that we can use a custom timeout. This is useful
  // when trying to detect network errors.
  this->socket->async_connect(*endpointIter++,
      common::weakBind(&Connection::OnConnect, this->shared_from_this(),
        boost::asio::placeholders::error, endpointIter));

  // Wait for at most 60 seconds for a connection to be established.
  // The connectionCondition notification occurs in ::OnConnect.
  if (!this->connectCondition.timed_wait(lock,
        boost::posix_time::milliseconds(60000)) || this->connectError)
  {
    gzlog << "Failed to create connection to remote host["
          << host << ":" << _port << "]\n";
    this->socket->close();
    return false;
  }

  if (this->remoteURI.empty())
  {
    gzerr << "Unable to connect to host[" << host << ":" << _port << "]\n";
    return false;
  }

  this->isOpen = true;

  return true;
}

//////////////////////////////////////////////////
void Connection::Listen(unsigned int port, const AcceptCallback &_acceptCB)
{
  this->acceptCB = _acceptCB;

  this->acceptor = new boost::asio::ip::tcp::acceptor(iomanager->GetIO());
  boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::tcp::v4(), port);
  this->acceptor->open(endpoint.protocol());
  this->acceptor->set_option(
      boost::asio::ip::tcp::acceptor::reuse_address(true));
  this->acceptor->set_option(
      boost::asio::ip::tcp::acceptor::keep_alive(true));

  // Enable TCP_NO_DELAY
  this->acceptor->set_option(boost::asio::ip::tcp::no_delay(true));

  this->acceptor->bind(endpoint);
  this->acceptor->listen();

  this->acceptConn = ConnectionPtr(new Connection());

  this->acceptor->async_accept(*this->acceptConn->socket,
      common::weakBind(&Connection::OnAccept, this->shared_from_this(),
                  boost::asio::placeholders::error));
}

//////////////////////////////////////////////////
void Connection::OnAccept(const boost::system::error_code &e)
{
  // Call the accept callback if there isn't an error
  if (!e)
  {
    this->acceptConn->isOpen = true;

    if (!this->ipWhiteList.empty() &&
        this->ipWhiteList.find("," +
          this->acceptConn->GetRemoteHostname() + ",") == std::string::npos)
    {
      gzlog << "Rejected connection from["
        << this->acceptConn->GetRemoteHostname() << "], not in white list["
        << this->ipWhiteList << "]\n";
    }
    else
    {
      this->acceptCB(this->acceptConn);
    }

    // First start a new acceptor
    this->acceptConn = ConnectionPtr(new Connection());

    this->acceptor->async_accept(*this->acceptConn->socket,
        common::weakBind(&Connection::OnAccept, this->shared_from_this(),
          boost::asio::placeholders::error));
  }
  else
  {
    // Probably the connection was closed. No need to report an error since
    // this can happen duing a shutdown.
    if (e.value() != ECANCELED)
      gzerr << e.message() << "Value[" << e.value() << "]" << std::endl;
  }
}

//////////////////////////////////////////////////
void Connection::StartRead(const ReadCallback & /*_cb*/)
{
  gzerr << "\n\n\n\n DONT USE \n\n\n\n";
}

//////////////////////////////////////////////////
void Connection::StopRead()
{
  this->readQuit = true;
}

//////////////////////////////////////////////////
void Connection::EnqueueMsg(const std::string &_buffer, bool _force)
{
  using namespace boost::placeholders;
  this->EnqueueMsg(_buffer, boost::bind(&dummy_callback_fn, _1), 0, _force);
}

//////////////////////////////////////////////////
void Connection::EnqueueMsg(const std::string &_buffer,
    boost::function<void(uint32_t)> _cb, uint32_t _id, bool _force)
{
  // Don't enqueue empty messages
  if (_buffer.empty() || !this->IsOpen())
  {
    return;
  }

  char headerBuffer[HEADER_LENGTH + 1];
  snprintf(headerBuffer, HEADER_LENGTH + 1, "%08x",
      static_cast<unsigned int>(_buffer.size()));

  {
    boost::recursive_mutex::scoped_lock lock(this->writeMutex);

    if (this->writeQueue.empty() ||
        (this->writeCount > 0 && this->writeQueue.size() == 1) ||
        (this->writeQueue.back().size() + HEADER_LENGTH + _buffer.size() >
         4096))
    {
      this->writeQueue.push_back(std::string(headerBuffer) + _buffer);
      this->callbacks.push_back({std::make_pair(_cb, _id)});
    }
    else
    {
      this->writeQueue.back() += std::string(headerBuffer) + _buffer;
      this->callbacks.back().push_back(std::make_pair(_cb, _id));
    }
  }

  if (_force)
  {
    this->ProcessWriteQueue();
  }
  else
  {
    // Tell the connection manager that it needs to update
    ConnectionManager::Instance()->TriggerUpdate();
  }
}

/////////////////////////////////////////////////
void Connection::ProcessWriteQueue(bool _blocking)
{
  boost::recursive_mutex::scoped_lock lock(this->writeMutex);

  if (!this->IsOpen())
  {
    return;
  }

  // async_write should only be called when the last async_write has
  // completed. therefore we have to check the writeCount attribute
  if (this->writeQueue.empty() || this->writeCount > 0)
  {
    return;
  }

  this->writeCount++;

  // Write the serialized data to the socket. We use
  // "gather-write" to send both the head and the data in
  // a single write operation
  if (!_blocking)
  {
    boost::asio::async_write(*this->socket,
        boost::asio::buffer(this->writeQueue.front().c_str(),
          this->writeQueue.front().size()),
          common::weakBind(&Connection::OnWrite, this->shared_from_this(),
            boost::asio::placeholders::error));
  }
  else
  {
    try
    {
      boost::asio::write(*this->socket,
          boost::asio::buffer(this->writeQueue.front().c_str(),
            this->writeQueue.front().size()));
    }
    catch(...)
    {
      this->Shutdown();
    }

    this->PostWrite();
  }
}

//////////////////////////////////////////////////
std::string Connection::GetLocalURI() const
{
  return this->localURI;
}

//////////////////////////////////////////////////
std::string Connection::GetRemoteURI() const
{
  return this->remoteURI;
}

//////////////////////////////////////////////////
void Connection::PostWrite()
{
  // Call the callbacks, if not NULL
  if (!this->callbacks.empty())
  {
    for (auto const &callback : this->callbacks.front())
      if (!callback.first.empty())
        callback.first(callback.second);
    this->callbacks.pop_front();
  }

  if (!this->writeQueue.empty())
    this->writeQueue.pop_front();
  this->writeCount--;
}

//////////////////////////////////////////////////
void Connection::OnWrite(const boost::system::error_code &_e)
{
  {
    boost::recursive_mutex::scoped_lock lock(this->writeMutex);

    this->PostWrite();
  }

  if (_e)
  {
    // It will reach this point if the remote connection disconnects.
    this->Shutdown();
  }
}

//////////////////////////////////////////////////
void Connection::Shutdown()
{
  if (!this->socket)
    return;

  this->Cancel();

  // Shutdown the TBB task
  this->shutdown();

  this->Close();
}

//////////////////////////////////////////////////
bool Connection::IsOpen() const
{
  bool result = this->socket && this->socket->is_open();
  return this->isOpen && result;
}

//////////////////////////////////////////////////
void Connection::Close()
{
  boost::mutex::scoped_lock lock(this->socketMutex);

  if (this->socket && this->socket->is_open())
  {
    try
    {
      this->socket->close();
      boost::system::error_code ec;
      this->socket->shutdown(boost::asio::ip::tcp::socket::shutdown_both, ec);
    }
    catch(boost::system::system_error &e)
    {
      // This warning message is unnecessary...
      // gzwarn << "Error closing socket[" << this->id << "] ["
             // << e.what() << "]\n";
    }

    delete this->socket;
    this->socket = NULL;
  }

  if (this->acceptor && this->acceptor->is_open())
  {
    try
    {
      this->acceptor->close();
    }
    catch(boost::system::system_error &e)
    {
      gzwarn <<"Error closing acceptor[" << this->id << "]\n";
    }

    delete this->acceptor;
    this->acceptor = NULL;
  }

  boost::recursive_mutex::scoped_lock lock2(this->writeMutex);
  this->writeQueue.clear();
  this->callbacks.clear();
}

//////////////////////////////////////////////////
void Connection::Cancel()
{
  if (this->acceptor)
  {
    try
    {
      this->acceptor->cancel();
    }
    catch(boost::system::system_error &e)
    {
      gzwarn << "Connection::Cancel Error[" << e.what() << "]\n";
      // Left empty on purpose
    }
    delete this->acceptor;
    this->acceptor = NULL;
  }

  {
    boost::mutex::scoped_lock lock(this->socketMutex);
    if (this->socket && this->socket->is_open())
    {
      try
      {
        this->socket->cancel();
      }
      catch(...)
      {
        this->socket->close();
      }
    }
  }
}

//////////////////////////////////////////////////
bool Connection::Read(std::string &data)
{
  bool result = false;
  char header[HEADER_LENGTH];
  std::vector<char> incoming;

  std::size_t incoming_size;
  boost::system::error_code error;

  boost::recursive_mutex::scoped_lock lock(this->readMutex);

  // First read the header
  this->socket->read_some(boost::asio::buffer(header), error);

  if (error)
  {
    gzerr << "Connection[" << this->id << "] Closed during Read\n";
    throw boost::system::system_error(error);
  }

  // Parse the header to get the size of the incoming data packet
  incoming_size = this->ParseHeader(std::string(header, HEADER_LENGTH));
  if (incoming_size > 0)
  {
    incoming.resize(incoming_size);

    std::size_t len = 0;
    do
    {
      // Read in the actual data
      len += this->socket->read_some(boost::asio::buffer(&incoming[len],
            incoming_size - len), error);
    } while (len < incoming_size && !error && !this->readQuit);

    if (len != incoming_size)
    {
      gzerr << "Did not read everying. Read[" << len
        << "] Needed[" << incoming_size << "]\n";
    }

    if (error)
      throw boost::system::system_error(error);

    data = std::string(&incoming[0], incoming.size());
    result = true;
  }

  return result;
}

//////////////////////////////////////////////////
std::string Connection::GetLocalAddress() const
{
  return this->localAddress;
}

//////////////////////////////////////////////////
unsigned int Connection::GetLocalPort() const
{
  try
  {
    if (this->socket && this->socket->is_open())
      return this->socket->local_endpoint().port();
    else if (this->acceptor)
      return this->acceptor->local_endpoint().port();
  }
  catch(...)
  {
  }

  return 0;
}

//////////////////////////////////////////////////
std::string Connection::GetRemoteAddress() const
{
  return this->remoteAddress;
}

//////////////////////////////////////////////////
unsigned int Connection::GetRemotePort() const
{
  if (this->socket && this->socket->is_open())
  {
    try
    {
      return this->socket->remote_endpoint().port();
    }
    catch(...)
    {
      return 0;
    }
  }
  else
    return 0;
}



//////////////////////////////////////////////////
std::size_t Connection::ParseHeader(const std::string &header)
{
  std::size_t data_size = 0;
  std::istringstream is(header);

  if (!(is >> std::hex >> data_size))
  {
    // Header doesn't seem to be valid. Inform the caller
    boost::system::error_code error(boost::asio::error::invalid_argument);
  }

  return data_size;
}

//////////////////////////////////////////////////
void Connection::ReadLoop(const ReadCallback &cb)
{
  std::string data;

  this->readQuit = false;
  while (!this->readQuit)
  {
    try
    {
      if (this->socket->available() >= HEADER_LENGTH)
      {
        if (this->Read(data))
        {
          (cb)(data);
        }
      }
      else
      {
        common::Time::MSleep(10);
        continue;
      }
    }
    catch(std::exception &e)
    {
      // The connection closed
      break;
    }
  }
}

//////////////////////////////////////////////////
boost::asio::ip::tcp::endpoint Connection::GetLocalEndpoint()
{
  boost::asio::ip::address_v4 address;

  // Get the GAZEBO_HOSTNAME environment variable. This will be NULL if it's not
  // set.
  char *hostname = getenv("GAZEBO_HOSTNAME");

  // Get the GAZEBO_IP environment variable. This will be NULL if it's not
  // set.
  char *ip = getenv("GAZEBO_IP");

  // First try GAZEBO_HOSTNAME if it is set.
  if (hostname && !std::string(hostname).empty())
  {
    boost::asio::ip::tcp::resolver resolver(iomanager->GetIO());
    boost::asio::ip::tcp::resolver::query query(hostname, "");
    boost::asio::ip::tcp::resolver::iterator iter = resolver.resolve(query);
    boost::asio::ip::tcp::resolver::iterator end;

    // Loop through the results, and stop at the first valid address.
    while (iter != end)
    {
      boost::asio::ip::tcp::endpoint testEndPoint = *iter++;

      // Check the end point for validity.
      if (!addressIsUnspecified(testEndPoint.address().to_v4()))
      {
        address = testEndPoint.address().to_v4();
        break;
      }
    }

    // Complain if GAZEBO_HOSTNAME was set, but we were not able to get
    // a valid address.
    if (addressIsUnspecified(address))
      gzerr << "GAZEBO_HOSTNAME[" << hostname << "] is invalid. "
            << "We will fallback onto GAZEBO_IP.";
  }

  // Try GAZEBO_IP if GAZEBO_HOSTNAME is not set or we were not able to
  // find a valid address.
  if (ip && !std::string(ip).empty() && addressIsUnspecified(address))
  {
    if (!ValidateIP(ip))
    {
      gzerr << "GAZEBO_IP environment variable with value[" << ip
            << "] is invalid. We will still try to use it, be warned.\n";
    }

    address = boost::asio::ip::address_v4::from_string(ip);
  }

  // Try to automatically find a valid address if GAZEBO_IP and
  // GAZEBO_HOSTNAME have failed.
  if (addressIsUnspecified(address))
  {
#ifndef _WIN32
    // the following is *nix implementation to get the external IP of the
    // current machine.

    struct ifaddrs *ifaddr, *ifa;

    // Get interface addresses
    if (getifaddrs(&ifaddr) == -1)
    {
      perror("getifaddres");
      gzthrow("Unable to get local interface addresses");
    }

    char host[NI_MAXHOST];

    // Iterate over all the interface addresses
    for (ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next)
    {
      // Only consider UP interfaces
      if (!(ifa->ifa_flags & IFF_UP))
        continue;
      if (ifa->ifa_addr == NULL)
        continue;

      int family = ifa->ifa_addr->sa_family;
      // \todo We currently don't handle AF_INET6 addresses. So I commented
      // out the below line, and removed AF_INET6 for the if clause.
      // if (family == AF_INET || family == AF_INET6)
      if (family == AF_INET)
      {
        int s = getnameinfo(ifa->ifa_addr,
            (family == AF_INET) ? sizeof(struct sockaddr_in) :
            sizeof(struct sockaddr_in6),
            host, NI_MAXHOST, NULL, 0, NI_NUMERICHOST);

        if (s != 0)
          gzthrow(std::string("getnameinfo() failed[") +
                  gai_strerror(s) + "]\n");

        // Validate the IP address to make sure it's a valid dotted quad.
        if (!ValidateIP(host))
          continue;

        address = boost::asio::ip::address_v4::from_string(host);

        // Also make sure that the IP address is not a loopback interface.
        if (!addressIsLoopback(address))
          break;
      }
    }

    // Use a loopback interface as a fallback.
    if (addressIsUnspecified(address))
    {
      gzwarn << "Unable to find a non-loopback interface. You will "
             << "not be able to connect to remote server.\n";

      address = address.loopback();
    }

    freeifaddrs(ifaddr);
// _WIN32
#else
  // Establish our default return value, in case everything below fails.
  std::string retAddr("127.0.0.1");

  // Look up our address.
  ULONG outBufLen = 0;
  PIP_ADAPTER_ADDRESSES addrs = NULL;

  // Not sure whether these are the right flags, but they work
  // on Windows 7
  ULONG flags = (GAA_FLAG_SKIP_ANYCAST | GAA_FLAG_SKIP_MULTICAST |
                 GAA_FLAG_SKIP_DNS_SERVER | GAA_FLAG_SKIP_FRIENDLY_NAME);

  // The first time, it'll fail; we're asking how much space is needed to
  // store the result.
  GetAdaptersAddresses(AF_INET, flags, NULL, addrs, &outBufLen);

  // Allocate the required space.
  addrs = new IP_ADAPTER_ADDRESSES[outBufLen];
  ULONG ret;

  // Now the call should succeed.
  if ((ret = GetAdaptersAddresses(AF_INET, flags, NULL, addrs, &outBufLen)) ==
      NO_ERROR)
  {
    // Iterate over all returned adapters, arbitrarily sticking with the
    // last non-loopback one that we find.
    for (PIP_ADAPTER_ADDRESSES curr = addrs; curr; curr = curr->Next)
    {
      // The interface is not running.
      if (curr->OperStatus != IfOperStatusUp)
        continue;

      // Iterate over all unicast addresses for this adapter
      for (PIP_ADAPTER_UNICAST_ADDRESS unicast = curr->FirstUnicastAddress;
           unicast; unicast = unicast->Next)
      {
        // Cast to get an IPv4 numeric address (the AF_INET flag used above
        // ensures that we're only going to get IPv4 address here).
        sockaddr_in* sockaddress =
          reinterpret_cast<sockaddr_in*>(unicast->Address.lpSockaddr);

        // Make it a dotted quad
        char ipv4Str[3*4+3+1];

        snprintf(ipv4Str, sizeof(ipv4Str), "%d.%d.%d.%d",
          sockaddress->sin_addr.S_un.S_un_b.s_b1,
          sockaddress->sin_addr.S_un.S_un_b.s_b2,
          sockaddress->sin_addr.S_un.S_un_b.s_b3,
          sockaddress->sin_addr.S_un.S_un_b.s_b4);

        // Ignore loopback address (that's our default anyway)
        if (!strcmp(ipv4Str, "127.0.0.1"))
          continue;

        retAddr = ipv4Str;
      }
    }
  }
  else
  {
    gzerr << "GetAdaptersAddresses() failed: " << ret << std::endl;
  }

  delete [] addrs;

  if (retAddr == "127.0.0.1")
  {
    gzwarn <<
      "Couldn't find a preferred IP via the GetAdaptersAddresses() call; "
      "I'm assuming that your IP "
      "address is 127.0.0.1.  This should work for local processes, "
      "but will almost certainly not work if you have remote processes."
      "Report to the disc-zmq development team to seek a fix." << std::endl;
  }
  address = boost::asio::ip::address_v4::from_string(retAddr);
#endif
  }

  // Complain if we were unable to find a valid address
  if (addressIsUnspecified(address))
    gzthrow("Unable to get IP address for the local machine."
            "Please check your network configuration.");

  return boost::asio::ip::tcp::endpoint(address, 0);
}

/////////////////////////////////////////////////
bool Connection::ValidateIP(const std::string &_ip)
{
  struct sockaddr_in sa;

#ifdef _WIN32
  int result = InetPton(AF_INET, _ip.c_str(), &(sa.sin_addr));
#else
  int result = inet_pton(AF_INET, _ip.c_str(), &(sa.sin_addr));
#endif
  return result != 0;
}

//////////////////////////////////////////////////
boost::asio::ip::tcp::endpoint Connection::GetRemoteEndpoint() const
{
  boost::asio::ip::tcp::endpoint ep;
  if (this->socket)
  {
    boost::system::error_code ec;
    ep = this->socket->remote_endpoint(ec);
    if (ec)
      gzerr << "Getting remote endpoint failed" << std::endl;
  }
  return ep;
}

//////////////////////////////////////////////////
std::string Connection::GetHostname(boost::asio::ip::tcp::endpoint _ep)
{
  std::string result;

  // Use the IP address if it's valid. This saves time, and is better than
  // trying to find a hostname (particularly in cases where /etc/hosts has
  // bad information).
  if (!addressIsUnspecified(_ep.address().to_v4()))
  {
    result = _ep.address().to_string();
  }
  // Otherwise perform a lookup
  else
  {
    boost::asio::ip::tcp::resolver resolver(iomanager->GetIO());
    boost::asio::ip::tcp::resolver::iterator iter = resolver.resolve(_ep);
    boost::asio::ip::tcp::resolver::iterator end;

    while (iter != end)
    {
      result = (*iter).host_name();
      ++iter;
    }
  }

  return result;
}

//////////////////////////////////////////////////
std::string Connection::GetRemoteHostname() const
{
  return this->GetHostname(this->GetRemoteEndpoint());
}

//////////////////////////////////////////////////
std::string Connection::GetLocalHostname()
{
  return GetHostname(GetLocalEndpoint());
}

//////////////////////////////////////////////////
void Connection::OnConnect(const boost::system::error_code &_error,
    boost::asio::ip::tcp::resolver::iterator /*_endPointIter*/)
{
  // This function is called when a connection is successfully (or
  // unsuccessfully) established.

  boost::mutex::scoped_lock lock(this->connectMutex);
  if (_error == boost::system::errc::success)
  {
    this->remoteURI = std::string("http://") + this->GetRemoteHostname()
      + ":" + boost::lexical_cast<std::string>(this->GetRemotePort());

    if (this->socket && this->socket->is_open())
    {
      this->remoteAddress =
        this->socket->remote_endpoint().address().to_string();
    }
    else
    {
      this->connectError = true;
      gzerr << "Invalid socket connection\n";
    }

    // Notify the condition that it may proceed.
    this->connectCondition.notify_one();
  }
  else
  {
    this->connectError = true;
    this->connectCondition.notify_one();
  }
}

//////////////////////////////////////////////////
unsigned int Connection::GetId() const
{
  return this->id;
}

//////////////////////////////////////////////////
std::string Connection::GetIPWhiteList() const
{
  return this->ipWhiteList;
}
