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
}
