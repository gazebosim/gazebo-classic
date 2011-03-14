#include "IOManager.hh"
#include "Connection.hh"

using namespace gazebo;
using namespace transport;

Connection::Connection(boost::asio::io_service &io_service)
  : socket(io_service)
{
}

Connection::~Connection()
{
}

boost::asio::ip::tcp::socket &Connection::GetSocket()
{
  return this->socket;
}

void Connection::Connect(const std::string &host, const std::string &service)
{
  // Resolve the host name into an IP address
  boost::asio::ip::tcp::resolver resolver(IOManager::Instance()->GetIO());
  boost::asio::ip::tcp::resolver::query query(host, service);
  boost::asio::ip::tcp::resolver::iterator endpoint_iter = resolver.resolve(query);
  boost::asio::ip::tcp::resolver::iterator end;
  //boost::asio::ip::tcp::endpoint endpoint = *endpoint_iter;

  boost::system::error_code error = boost::asio::error::host_not_found;
  while (error && endpoint_iter != end)
  {
    socket.close();
    socket.connect(*endpoint_iter++, error);
  }
  if (error)
    throw boost::system::system_error(error);
}
