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
#include <boost/lexical_cast.hpp>

#include "common/Console.hh"
#include "common/Messages.hh"

#include "transport/IOManager.hh"
#include "transport/Connection.hh"

using namespace gazebo;
using namespace transport;

unsigned int Connection::idCounter = 0;

////////////////////////////////////////////////////////////////////////////////
// Constructor
Connection::Connection()
  : socket( IOManager::Instance()->GetIO() )
{
  IOManager::Instance()->IncCount();
  this->id = idCounter++;

  //std::cout << "Connection.cc Debug: " << this->id << "\n";

  this->writeMutex = new boost::mutex();
  this->acceptor = NULL;
  this->readThread = NULL;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Connection::~Connection()
{
  this->StopRead();
  this->Close();
  this->Cancel();

  delete this->writeMutex;
  IOManager::Instance()->DecCount();
}

////////////////////////////////////////////////////////////////////////////////
// Connect to a remote host
void Connection::Connect(const std::string &host, unsigned short port)
{
  std::string service = boost::lexical_cast<std::string>(port);

  // Resolve the host name into an IP address
  boost::asio::ip::tcp::resolver resolver(IOManager::Instance()->GetIO());
  boost::asio::ip::tcp::resolver::query query(host, service);
  boost::asio::ip::tcp::resolver::iterator endpoint_iter = resolver.resolve(query);
  boost::asio::ip::tcp::resolver::iterator end;

  boost::system::error_code error = boost::asio::error::host_not_found;

  while (error && endpoint_iter != end)
  {
    this->socket.close();
    this->socket.connect(*endpoint_iter++, error);
  }

  if (error)
    gzthrow ("Unable to connect to " << host << ":" << port);
}

////////////////////////////////////////////////////////////////////////////////
/// \brief Start a server that listens on a port
void Connection::Listen(unsigned short port, const AcceptCallback &accept_cb)
{
  this->acceptCB = accept_cb;

  this->acceptor = new boost::asio::ip::tcp::acceptor(IOManager::Instance()->GetIO());
  boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::tcp::v4(), port);
  this->acceptor->open(endpoint.protocol());
  this->acceptor->set_option(boost::asio::ip::tcp::acceptor::reuse_address(true));
  this->acceptor->bind(endpoint);
  this->acceptor->listen();

  this->acceptConn = ConnectionPtr(new Connection());

  this->acceptor->async_accept(this->acceptConn->socket,
      boost::bind(&Connection::OnAccept, this, 
                  boost::asio::placeholders::error));
}

////////////////////////////////////////////////////////////////////////////////
// Accept a new connection to this server, and start a new acceptor
void Connection::OnAccept(const boost::system::error_code &e)
{
  // Call the accept callback if there isn't an error
  if (!e)
  {
    // First start a new acceptor
    this->acceptCB( this->acceptConn );

    this->acceptConn = ConnectionPtr(new Connection());

    this->acceptor->async_accept(this->acceptConn->socket, 
        boost::bind(&Connection::OnAccept, this, 
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

////////////////////////////////////////////////////////////////////////////////
/// Start a thread that reads from the connection, and passes
/// new message to the ReadCallback
void Connection::StartRead(const ReadCallback &cb)
{
  this->readThread = new boost::thread( boost::bind( &Connection::ReadLoop, this, cb ) ); 
}

////////////////////////////////////////////////////////////////////////////////
/// Stop the read loop 
void Connection::StopRead()
{
  if (this->readThread)
  {
    this->readThread->interrupt();
    delete this->readThread;
  }
  this->readThread = NULL;
}

////////////////////////////////////////////////////////////////////////////////
// Write data out
void Connection::EnqueueMsg(const std::string &buffer, bool force)
{
  std::ostringstream header_stream;
  header_stream << std::setw(HEADER_LENGTH) << std::hex << buffer.size();

  if (!header_stream || header_stream.str().size() != HEADER_LENGTH)
  {
    //Something went wrong, inform the caller
    boost::system::error_code error(boost::asio::error::invalid_argument);
    gzerr << "Connection::Write error[" << error.message() << "]\n";
    return;
  }

  boost::mutex::scoped_lock( *this->writeMutex );
  this->writeQueue.push_back(header_stream.str());
  this->writeQueue.push_back(buffer);

  if (force)
    this->ProcessWriteQueue();
}

void Connection::ProcessWriteQueue()
{
  boost::mutex::scoped_lock( *this->writeMutex );

  if (this->writeQueue.size() > 0)
  {
    unsigned int sum = 0;
    unsigned int i = 0;
    for (; i < this->writeCounts.size(); i++)
      sum += this->writeCounts[i];

    if (sum < this->writeQueue.size())
    {
      std::list<boost::asio::const_buffer> buffer;

      for (i=sum; i < this->writeQueue.size(); i++)
        buffer.push_back( boost::asio::buffer( this->writeQueue[i] ) );
      this->writeCounts.push_back( buffer.size() );

      // Write the serialized data to the socket. We use
      // "gather-write" to send both the head and the data in
      // a single write operation
      boost::asio::async_write( this->socket, buffer, 
          boost::bind(&Connection::OnWrite, this, 
            boost::asio::placeholders::error));
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Get the local URI
std::string Connection::GetLocalURI() const
{
  return "http://" + this->GetLocalHostname() + ":" + boost::lexical_cast<std::string>(this->GetLocalPort()); 
}
              
////////////////////////////////////////////////////////////////////////////////
/// Get the remote URI
std::string Connection::GetRemoteURI() const
{
  return "http://" + this->GetRemoteHostname() + ":" + boost::lexical_cast<std::string>(this->GetRemotePort()); 
}


////////////////////////////////////////////////////////////////////////////////
// Handle on write callbacks
void Connection::OnWrite(const boost::system::error_code &e)
{
  if (e)
    throw boost::system::system_error(e);
  else
  {
    boost::mutex::scoped_lock( *this->writeMutex );
    for (unsigned int i=0; i < this->writeCounts[0]; i++)
      this->writeQueue.pop_front();

    this->writeCounts.pop_front();
  }
}

////////////////////////////////////////////////////////////////////////////////
// Close a connection
void Connection::Close()
{
  if (this->socket.is_open())
    this->socket.close();

  if (this->acceptor && this->acceptor->is_open())
    this->acceptor->close();
}

////////////////////////////////////////////////////////////////////////////////
// Cancel all async operations on an open socket
void Connection::Cancel()
{
  if (this->acceptor)
  {
    try
    {
      this->acceptor->cancel();
    } 
    catch (boost::system::system_error &e)
    { }
    delete this->acceptor;
    this->acceptor = NULL;
  }

  if (this->socket.is_open())
    this->socket.cancel();
}

////////////////////////////////////////////////////////////////////////////////
// Read data from the socket
void Connection::Read(std::string &data)
{

  char header[HEADER_LENGTH];
  std::vector<char> incoming;

  std::size_t incoming_size;
  boost::system::error_code error;

  // First read the header
  this->socket.read_some( boost::asio::buffer(header), error );
  if (error)
  {
    gzerr << "Connection[" << this->id << "] Closed during Read\n";
    throw boost::system::system_error(error);
  }

  // Parse the header to get the size of the incoming data packet
  incoming_size = this->ParseHeader( header );

  incoming.resize( incoming_size );

  // Read in the actual data
  this->socket.read_some( boost::asio::buffer(incoming), error );
  if (error)
    throw boost::system::system_error(error);

  data = std::string(&incoming[0], incoming.size());
}

////////////////////////////////////////////////////////////////////////////////
/// Get the address of this connection
std::string Connection::GetLocalAddress() const
{
  return this->GetLocalEndpoint().address().to_string();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the port of this connection
unsigned short Connection::GetLocalPort() const
{
  if (this->socket.is_open())
    return this->socket.local_endpoint().port();
  else if (this->acceptor)
    return this->acceptor->local_endpoint().port();
  else
    gzerr << "No socket is open, unable to get port information.\n";

  return 0;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the remote address
std::string Connection::GetRemoteAddress() const
{
  if (this->socket.is_open())
    return this->socket.remote_endpoint().address().to_string();
  else
    return "";
}

////////////////////////////////////////////////////////////////////////////////
/// Get the remote port number
unsigned short Connection::GetRemotePort() const
{
  if (this->socket.is_open())
    return this->socket.remote_endpoint().port();
  else
    return 0;
}



////////////////////////////////////////////////////////////////////////////////
// Parse a header to get the size of a packet
std::size_t Connection::ParseHeader( const std::string &header )
{
  std::size_t data_size = 0;

  std::istringstream is(header);

  if (!(is >> std::hex >> data_size))
  {
    // Header doesn't seem to be valid. Inform the caller
    boost::system::error_code error(boost::asio::error::invalid_argument);
    std::ostringstream stream;
    stream << "Invalid header[" << error.message() << "] Data Size[" << data_size << "]";
    gzthrow(stream.str());
  }

  return data_size;
}

////////////////////////////////////////////////////////////////////////////////
/// the read thread
void Connection::ReadLoop(const ReadCallback &cb)
{
  fd_set fileDescriptorSet;
  struct timeval timeStruct;
  std::string data;

  int nativeSocket = this->socket.native();

  timeStruct.tv_sec = 1;
  timeStruct.tv_usec = 0;

  while (!this->readQuit)
  {
    try
    {
      boost::this_thread::interruption_point();
      if (this->socket.available() >= HEADER_LENGTH)
      {
        this->Read(data);
        (cb)(data);
      }
      else
      {
        usleep(33000);
        continue;
      }
    }
    catch (std::exception &e)
    {
      // The connection closed
      break;
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
// Get the local endpoint
boost::asio::ip::tcp::endpoint Connection::GetLocalEndpoint() const
{
  boost::asio::ip::tcp::resolver resolver(IOManager::Instance()->GetIO());
  boost::asio::ip::tcp::resolver::query query(boost::asio::ip::host_name(), "");
  boost::asio::ip::tcp::resolver::iterator iter = resolver.resolve(query);
  boost::asio::ip::tcp::resolver::iterator end; // End marker.
  boost::asio::ip::tcp::endpoint ep;

  while (iter != end)
    ep = *iter++;

  return ep;
}

boost::asio::ip::tcp::endpoint Connection::GetRemoteEndpoint() const
{
  //if (this->socket.is_open())
    return this->socket.remote_endpoint();
}

std::string Connection::GetHostname(boost::asio::ip::tcp::endpoint ep) const
{
  boost::asio::ip::tcp::resolver resolver(IOManager::Instance()->GetIO());
  boost::asio::ip::tcp::resolver::iterator iter = resolver.resolve(ep);
  boost::asio::ip::tcp::resolver::iterator end;

  std::string name;

  while (iter != end)
  {
    name = (*iter).host_name();
    ++iter;
  } 

  return name;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the remote hostname
std::string Connection::GetRemoteHostname() const
{
  return this->GetHostname( this->GetRemoteEndpoint() );
}

////////////////////////////////////////////////////////////////////////////////
/// Get the local hostname
std::string Connection::GetLocalHostname() const
{
  return this->GetHostname( this->GetLocalEndpoint() );
}


