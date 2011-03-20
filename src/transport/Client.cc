#include <boost/bind.hpp>

#include "common/Messages.hh"
#include "transport/IOManager.hh"
#include "transport/Client.hh"

using namespace gazebo;
using namespace transport;

Client::Client( const std::string &host, const std::string &service)
  //: connection(IOManager::Instance()->GetIO())
{
  std::cout << "transport::Client::Constructor\n";
  this->host = host;
  this->service = service;

  this->connection.reset( new Connection(IOManager::Instance()->GetIO()) );

  this->connection->Connect(this->host, this->service);

  // Resolve the host name into an IP address
  /*boost::asio::ip::tcp::resolver resolver(IOManager::Instance()->GetIO());
  boost::asio::ip::tcp::resolver::query query(host, service);
  boost::asio::ip::tcp::resolver::iterator endpoint_iter = resolver.resolve(query);
  boost::asio::ip::tcp::endpoint endpoint = *endpoint_iter;

  // Start an asynchronous connect operation
  this->connection.GetSocket().async_connect( endpoint,
      boost::bind(&Client::OnConnect, this,
        boost::asio::placeholders::error, ++endpoint_iter) );
        */
}

void Client::OnConnect(const boost::system::error_code &error,
    boost::asio::ip::tcp::resolver::iterator endpoint_iter)
{
  std::cout << "Client::OnConnect\n";
  if (!error)
  {
    // Successful connection. Get the list of topics
    this->connection->Read( boost::bind(&Client::OnReadInit, this, _1) );
  }

  /*if (!error)
  {
    // Successful connection. Get the list of topics
    this->connection.async_read( boost::bind(&Client::OnReadInit, this, _1) );
  }
  else if (endpoint_iter != boost::asio::ip::tcp::resolver::iterator())
  {
    // try the next endpoint
    this->connection.GetSocket().close();
    boost::asio::ip::tcp::endpoint endpoint = *endpoint_iter;

    // Start an asynchronous connect operation
    this->connection.GetSocket().async_connect( endpoint,
        boost::bind(&Client::OnConnect, this,
          boost::asio::placeholders::error, ++endpoint_iter) );
  }
  else
  {
    // An error occurred
    std::cerr << error.message() << std::endl;
  }
  */
}

void Client::OnReadInit(const std::vector<char> &data)
{
  std::cout << "Client::OnReadInit\n";
  /*IntMapMessage msg;
  Message::fillFromBuffer(msg, data);

  this->topics.clear();

  std::map<std::string, int>::iterator iter;
  for (iter = msg.data.begin(); iter != msg.data.end(); iter++)
  {
    std::cout << "Topic[" << iter->first << "] Type[" << iter->second << "]\n";
    this->topics[iter->first] = Message::ValidType(iter->second); 
  }
  */
}

void Client::OnRead(const std::vector<char> &data)
{
  std::cout << "Client::OnRead\n";
  //this->connection.async_read( boost::bind(&Client::OnRead, this, _1) );
  //this->callback->OnRead(data);
}
