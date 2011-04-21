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

////////////////////////////////////////////////////////////////////////////////
// Constructor
Connection::Connection()
  : socket( IOManager::Instance()->GetIO() )
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Connection::~Connection()
{
  this->Cancel();
  this->Close();
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

  ConnectionPtr newConnection(new Connection());

  this->acceptor->async_accept(newConnection->socket,
      boost::bind(&Connection::OnAccept, this, 
                  boost::asio::placeholders::error, newConnection));
}

////////////////////////////////////////////////////////////////////////////////
// Accept a new connection to this server, and start a new acceptor
void Connection::OnAccept(const boost::system::error_code &e, 
                          ConnectionPtr conn)
{
  // First start a new acceptor
  ConnectionPtr newConnection(new Connection());
  this->acceptor->async_accept(newConnection->socket, 
      boost::bind(&Connection::OnAccept, this, 
        boost::asio::placeholders::error, newConnection));

  // Call the accept callback if there isn't an error
  if (!e)
    this->acceptCB( conn );
  else
    gzerr << e.message() << std::endl;
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
    this->readThread->interrupt();
  delete this->readThread;
  this->readThread = NULL;
}

////////////////////////////////////////////////////////////////////////////////
// Write data out
void Connection::Write(const std::string &buffer)
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

  // Keep a copy of the data valid during the write operation
  this->outbound_header = header_stream.str();
  this->outbound_data = buffer;

  // Write the serialized data to the socket. We use
  // "gather-write" to send both the head and the data in
  // a single write operation
  std::vector<boost::asio::const_buffer> buffers;
  buffers.push_back(boost::asio::buffer(this->outbound_header));
  buffers.push_back(boost::asio::buffer(this->outbound_data));
  boost::asio::async_write( this->socket, buffers, boost::bind(&Connection::OnWrite, this, boost::asio::placeholders::error));
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
}

////////////////////////////////////////////////////////////////////////////////
// Close a connection
void Connection::Close()
{
  if (this->socket.is_open())
    this->socket.close();
}

////////////////////////////////////////////////////////////////////////////////
// Cancel all async operations on an open socket
void Connection::Cancel()
{
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
    throw boost::system::system_error(error);

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
  std::string data;
  while (!this->readQuit)
  {
    try
    {
      this->Read(data);
    }
    catch (std::exception &e)
    {
      // The connection closed
      break;
    }
    (cb)(data);
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


