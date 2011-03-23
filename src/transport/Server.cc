#include <boost/serialization/vector.hpp>
#include <boost/lexical_cast.hpp>

#include "common/Messages.hh"
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

void Server::ProcessIncoming()
{
  unsigned int i,j,k;
  std::string msgData, type;
  msgs::Packet packet;

  // Process each connection
  for (i=0; i < this->connections.size(); i++)
  {
    unsigned int msgCount = this->connections[i]->GetReadBufferSize();

    // Process each buffered incoming message on the connection
    for (j=0; j < msgCount; j++)
    {
      // Get the next message on the connection
      this->connections[i]->PopReadBuffer(msgData);
      packet.ParseFromString( msgData );

      type = packet.type();

      std::cout << "Server got type[" << type << "]\n";

      // Send the message to all subscribers
      for (k=0; k < this->subscriptions[type].size(); k++)
      {
        this->subscriptions[type][k]->HandleMessage(packet.serialized_data());
      }
    }
  }
}

void Server::OnAccept(const boost::system::error_code &e, ConnectionPtr conn)
{
  std::cout << "Server on accept\n";
  if (!e)
  {
    this->connections.push_back(conn);

    // Spin up a thread that reads incoming data on the connection
    conn->StartReadThread();

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

void Server::Write(const google::protobuf::Message &msg,
                   std::string connection_address, unsigned short port)
{
  for (unsigned int i=0; i < this->connections.size(); i++)
  {
    if (this->connections[i]->GetRemoteAddress() == connection_address &&
        this->connections[i]->GetRemotePort() == port)
    {
      std::cout << "Server write message[" << msg.DebugString() << "]\n";
      this->connections[i]->Write( common::Message::Package("pub_link", msg) );
    }
  }
}

unsigned int Server::GetConnectionCount() const
{
  return this->connections.size();
}
