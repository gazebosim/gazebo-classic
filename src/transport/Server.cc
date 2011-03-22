#include <boost/serialization/vector.hpp>
#include <boost/lexical_cast.hpp>

#include "transport/IOManager.hh"
#include "transport/Server.hh"
#include "gazebo_config.h"

using namespace gazebo;
using namespace transport;

Server::Server(unsigned short port)
  : acceptor(IOManager::Instance()->GetIO(), 
             boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), port))
{
  this->hostname = boost::asio::ip::host_name();
  this->port = port;

  std::cout << "Server started[" << this->hostname << ":" << this->port<<"]\n";

  // Create a connection on which to accept incomming connections
  ConnectionPtr new_conn(new Connection(this->acceptor.io_service()));
  this->acceptor.async_accept(new_conn->GetSocket(),
      boost::bind(&Server::OnAccept, this, 
                  boost::asio::placeholders::error, new_conn));
}

void Server::OnAccept(const boost::system::error_code &e, ConnectionPtr conn)
{
  if (!e)
  {
    this->connections.push_back(conn);

    msgs::String stringMsg;
    stringMsg.set_data( std::string("gazebo ") + GAZEBO_VERSION );

    std::cout << "Server::OnAccept RemoteAddress[" << conn->GetRemoteAddress() << ":" << conn->GetRemotePort() << "]\n";
    conn->Write( stringMsg );

    // Start an accept operation for a new connection
    ConnectionPtr new_conn(new Connection(this->acceptor.io_service()));
    this->acceptor.async_accept(new_conn->GetSocket(),
        boost::bind(&Server::OnAccept, this, 
                    boost::asio::placeholders::error, new_conn));
  }
  else
  {
    // An error occurred. Log it and return.
    std::cerr << e.message() << std::endl;
  }
}

void Server::Write(const google::protobuf::Message &msg)
{
  for (unsigned int i=0; i < this->connections.size(); i++)
  {
    this->connections[i]->Write(msg);
  }
}

/*void Server::Publish(const StringMessage &msg)
{
  std::vector<ConnectionPtr>::iterator iter;
  for (iter = this->connections.begin(); iter != this->connections.end(); iter++)
  {
    printf("Writing to..\n");
     (*iter)->write(msg, boost::bind(&Server::OnWrite, this,
                          boost::asio::placeholders::error, (*iter)));
  }
}*/

int Server::GetConnectionCount() const
{
  return this->connections.size();
}
