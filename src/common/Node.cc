#include "transport/IOManager.hh"

#include "gazebo_config.h"
#include "common/Node.hh"

using namespace gazebo;
using namespace common;

Node::Node()
  : masterConn( new transport::Connection() ),
    serverConn( new transport::Connection() )
{
  transport::IOManager::Instance()->Start();
}

Node::~Node()
{
  transport::IOManager::Instance()->Stop();
}

void Node::Init(const std::string &master_host, unsigned short master_port)
{
  // Create a new TCP server on a free port
  this->serverConn->Listen(0, boost::bind(&Node::OnAccept, this, _1) );

  this->masterConn->Connect(master_host, master_port);

  std::string initData;
  this->masterConn->Read(initData);
  msgs::Packet packet;
  packet.ParseFromString(initData);

  if (packet.type() == "init")
  {
    msgs::String msg;
    msg.ParseFromString( packet.serialized_data() );
    if (msg.data() == std::string("gazebo ") + GAZEBO_VERSION)
    {
      // TODO: set some flag.. maybe start "serverConn" when initialized
      std::cout << "INITIALIZED...I should do something with this info\n";
    }
    else
    {
      // TODO: MAke this a proper error
      std::cerr << "Conflicting gazebo versions\n";
    }
  }
  else
    std::cerr << "Didn't receive an init from the master\n";

  this->masterConn->StartRead( boost::bind(&Node::OnMasterRead, this, _1) );
}

void Node::OnMasterRead( const std::string &data)
{
  msgs::Packet packet;
  packet.ParseFromString(data);

  if (packet.type() == "publisher_update")
  {
    msgs::Publish pub;
    pub.ParseFromString( packet.serialized_data() );

    std::string remoteHost = pub.host();
    unsigned short remotePort = pub.port();

    // Connect to the remote publisher.
    transport::ConnectionPtr connection(new transport::Connection());
    connection->Connect(remoteHost,remotePort);

    transport::TopicManager::Instance()->UpdatePublications(pub.topic(), pub.msg_type());

    transport::TopicManager::Instance()->ConnectSubToPub( pub, connection );

    msgs::Subscribe subscribeMsg;
    subscribeMsg.set_topic( pub.topic() );
    subscribeMsg.set_msg_type( pub.msg_type() );
    subscribeMsg.set_host( this->serverConn->GetLocalAddress() );
    subscribeMsg.set_port( this->serverConn->GetLocalPort() );

    // Tell the remote publisher we are subscribing
    connection->Write( Message::Package("sub", subscribeMsg) );

    // Save this connection
    this->subConnections.push_back(connection);
  }
  else
    std::cerr << "Node::OnMasterRead unknown type[" << packet.type() << "]\n";
}

void Node::OnAccept(const transport::ConnectionPtr &new_connection)
{
  std::cout << "Node accept... received a new subscriber??\n";
  new_connection->AsyncRead(
      boost::bind(&Node::OnReadHeader, this, new_connection, _1) );
}

void Node::OnReadHeader(const transport::ConnectionPtr &connection,
                        const std::string &data )
{
  msgs::Packet packet;
  packet.ParseFromString(data);

  std::cout << "Node::OnReadHeader" << packet.DebugString();

  if (packet.type() == "sub")
  {
    msgs::Subscribe sub;
    sub.ParseFromString( packet.serialized_data() );

    transport::TopicManager::Instance()->ConnectPubToSub(sub, connection);

    std::cout << "Create a publication transport\n";
  }
  else
    std::cerr << "Error est here\n";
}
