#include "transport/Client.hh"
#include "transport/IOManager.hh"

#include "common/Node.hh"

using namespace gazebo;
using namespace common;

Node::Node()
  : connection( new transport::Connection() )
{
  transport::IOManager::Instance()->Start();
}

Node::~Node()
{
  transport::IOManager::Instance()->Stop();
}

void Node::Init(const std::string &master_host, unsigned short master_port)
{
  this->connection->Connect(master_host, master_port);
  this->connection->StartRead( boost::bind( &Node::OnRead, this, _1, _2 ) );
}

void Node::OnRead(const transport::ConnectionPtr &conn,const std::string &data)
{
  msgs::Packet packet;
  packet.ParseFromString(data);

  if (packet.type() == "publisher")
  {
    msgs::Publish pub;
    pub.ParseFromString( packet.serialized_data() );
    transport::TopicManager::Instance()->ConnectSubscriber( pub );
  }
}
