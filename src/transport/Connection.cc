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
#include "msgs/msgs.h"

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
  this->debug = false;

  gzdbg << "Creating Connection[" << this->id << "]\n";

  this->writeMutex = new boost::recursive_mutex();
  this->acceptor = NULL;
  this->readThread = NULL;
  this->readQuit = false;
  this->writeQueue.clear();
  //boost::asio::socket_base::send_buffer_size(8192*1);
  //boost::asio::socket_base::receive_buffer_size(8192*1);
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Connection::~Connection()
{
  gzdbg << "Deleting Connection[" << this->id << "]\n";
  this->Shutdown();
  this->writeQueue.clear();

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
      boost::bind(&Connection::OnAccept, shared_from_this(), 
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
        boost::bind(&Connection::OnAccept, shared_from_this(), 
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
  this->readThread = new boost::thread( boost::bind( &Connection::ReadLoop, shared_from_this(), cb ) ); 
}

////////////////////////////////////////////////////////////////////////////////
/// Stop the read loop 
void Connection::StopRead()
{
  gzdbg << "Connection Stop Read[" << this->id << "]\n";
  this->readQuit = true;
  if (this->readThread)
  {
    gzdbg << "Deleting the thread\n";
    this->readThread->interrupt();
    delete this->readThread;
  }
  this->readThread = NULL;
  gzdbg << "Connection DONE Stop Read[" << this->id << "]\n";
}

////////////////////////////////////////////////////////////////////////////////
// Write data out
void Connection::EnqueueMsg(const std::string &_buffer, bool _force)
{
  this->writeMutex->lock();

  std::ostringstream header_stream;

  header_stream << std::setw(HEADER_LENGTH) << std::hex << _buffer.size();

  if ( header_stream.str().empty() ||
      header_stream.str().size() != HEADER_LENGTH)
  {
    //Something went wrong, inform the caller
    boost::system::error_code error(boost::asio::error::invalid_argument);
    gzerr << "Connection::Write error[" << error.message() << "]\n";
    this->writeMutex->unlock();
    return;
  }

  this->writeQueue.push_back(header_stream.str());
  this->writeQueue.push_back(_buffer);

  if (_force)
    this->ProcessWriteQueue();

  this->writeMutex->unlock();
}

void Connection::ProcessWriteQueue()
{
  this->writeMutex->lock();

  if (this->writeQueue.size() > 0)
  {
    boost::asio::streambuf *buffer = new boost::asio::streambuf;
    std::ostream os(buffer);

    for (unsigned int i=0; i < this->writeQueue.size(); i++)
    {
      os << this->writeQueue[i];
    }
    this->writeQueue.clear();

    // Write the serialized data to the socket. We use
    // "gather-write" to send both the head and the data in
    // a single write operation
    boost::asio::async_write( this->socket, buffer->data(), 
        boost::bind(&Connection::OnWrite, shared_from_this(), 
          boost::asio::placeholders::error, buffer));

  }
  this->writeMutex->unlock();
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
void Connection::OnWrite(const boost::system::error_code &e, 
    boost::asio::streambuf *_buffer)
{
  delete _buffer;

  if (e)
  {
    // It will reach this point if the remote connection disconnects.
    this->Shutdown();
  }
}

////////////////////////////////////////////////////////////////////////////////
// Shutdown the socket
void Connection::Shutdown()
{
  this->shutdownSignal();
  this->StopRead();

  if (this->socket.is_open())
  {
    boost::system::error_code ec;
    this->socket.shutdown(boost::asio::ip::tcp::socket::shutdown_both, ec);
  }

  //this->Cancel();
  this->Close();
  //this->acceptor = NULL;
  //this->readThread = NULL;
}

////////////////////////////////////////////////////////////////////////////////
/// Return true if the connection is open
bool Connection::IsOpen() const
{
  return this->socket.is_open();
}


////////////////////////////////////////////////////////////////////////////////
// Close a connection
void Connection::Close()
{
  if (this->socket.is_open())
  {
    try
    {
      this->socket.close();
    }
    catch (boost::system::system_error &e)
    {
      gzwarn <<"Error closing socket[" << this->id << "]\n";// msg[" << e.what() << "]\n";
    }
  }

  if (this->acceptor && this->acceptor->is_open())
  {
    try
    {
      this->acceptor->close();
    }
    catch (boost::system::system_error &e)
    {
      gzwarn <<"Error closing acceptor[" << this->id << "]\n";// msg[" << e.what() << "]\n";
    }

  }
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
    { 
      gzwarn << "Connection::Cancel Error[" << e.what() << "]\n";
      // Left empty on purpose 
    }
    delete this->acceptor;
    this->acceptor = NULL;
  }

  if (this->socket.is_open())
    this->socket.cancel();
}

////////////////////////////////////////////////////////////////////////////////
// Read data from the socket
bool Connection::Read(std::string &data)
{
  bool result = false;
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
  if (incoming_size > 0)
  {
    incoming.resize( incoming_size );

    std::size_t len = 0;
    do
    {
      // Read in the actual data
      len += this->socket.read_some(boost::asio::buffer(&incoming[len], incoming_size - len), error);
    } while ( len < incoming_size && !error && !this->readQuit);

    if (len != incoming_size)
      gzerr << "Did not read everying. Read[" << len << "] Needed[" << incoming_size << "]\n";

    if (error)
      throw boost::system::system_error(error);

    data = std::string(&incoming[0], incoming.size());
    result = true;
  }

  return result;
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
    gzerr << "Invalid header[" << error.message() << "] Data Size[" 
           << data_size << "] on Connection[" << this->id << "]. Header[" << header << "]\n";
    //gzthrow(stream.str());
  }

  return data_size;
}

////////////////////////////////////////////////////////////////////////////////
/// the read thread
void Connection::ReadLoop(const ReadCallback &cb)
{
  struct timeval timeStruct;
  std::string data;

  timeStruct.tv_sec = 1;
  timeStruct.tv_usec = 0;

  this->readQuit = false;
  while (!this->readQuit)
  {
    try
    {
      boost::this_thread::interruption_point();
      if (this->socket.available() >= HEADER_LENGTH)
      {
        if (this->Read(data))
        {
          (cb)(data);
        }
      }
      else
      {
        usleep(10000);
        continue;
      }
    }
    catch (std::exception &e)
    {
      // The connection closed
      break;
    }
  }

  gzdbg << "Ending ReadLoop[" << this->id << "]\n";
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


