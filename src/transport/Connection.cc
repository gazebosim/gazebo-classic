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

#include "common/Messages.hh"

#include "transport/IOManager.hh"
#include "transport/Connection.hh"

using namespace gazebo;
using namespace transport;

Connection::Connection()
  : socket( IOManager::Instance()->GetIO() )
{
  //this->readBufferMutex = new boost::mutex();
}

Connection::~Connection()
{
  std::cout << "Connection Destructor\n";
  //delete this->readBufferMutex;
  //this->readBufferMutex = NULL;
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
    throw boost::system::system_error(error);
}

////////////////////////////////////////////////////////////////////////////////
/// \brief Start a server that listens on a port
void Connection::Listen(unsigned short port, const AcceptCallback &accept_cb)
{
  this->acceptCB = accept_cb;

  std::cout << "Connection::Listen\n";
  this->acceptor = new boost::asio::ip::tcp::acceptor(
      IOManager::Instance()->GetIO(), 
      boost::asio::ip::tcp::endpoint( boost::asio::ip::tcp::v4(), port ));

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
    std::cerr << e.message() << std::endl;
}

////////////////////////////////////////////////////////////////////////////////
/// Start a thread that reads from the connection, and passes
/// new message to the ReadCallback
void Connection::StartRead(const ReadCallback &cb)
{
  this->readThread = new boost::thread( boost::bind( &Connection::ReadLoop, this, cb ) ); 
}

void Connection::Write(const google::protobuf::Message &msg)
{
  std::string out;

  if (!msg.SerializeToString(&out))
    gzthrow("Failed to serialized message");

  this->Write(out);
}

void Connection::Write(const std::string &buffer)
{
  std::ostringstream header_stream;
  header_stream << std::setw(HEADER_LENGTH) << std::hex << buffer.size();

  if (!header_stream || header_stream.str().size() != HEADER_LENGTH)
  {
    //Something went wrong, inform the caller
    boost::system::error_code error(boost::asio::error::invalid_argument);
    std::cerr << "Connection::Write error[" << error.message() << "]\n";
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
// Handle on write callbacks
void Connection::OnWrite(const boost::system::error_code &e)
{
  if (e)
    throw boost::system::system_error(e);
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
  std::cout << "Got a header Size[" << incoming_size << "]\n";
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
  if (this->socket.is_open())
    return this->socket.local_endpoint().address().to_string();
  else if (this->acceptor && this->acceptor->is_open())
    return this->acceptor->local_endpoint().address().to_string();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the port of this connection
unsigned short Connection::GetLocalPort() const
{
  if (this->socket.is_open())
    return this->socket.local_endpoint().port();
  else if (this->acceptor && this->acceptor->is_open())
    return this->acceptor->local_endpoint().port();
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
    gzthrow("Invalid header[" + error.message() + "]");
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
    this->Read(data);
    (cb)(data);
  }
}


