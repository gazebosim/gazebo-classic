#include "transport/Client.hh"
#include "transport/IOManager.hh"

#include "common/Node.hh"

using namespace gazebo;
using namespace common;

Node::Node()
{
  transport::IOManager::Instance()->Start();
  this->client = NULL;
}

Node::~Node()
{
  if (this->client)
    delete this->client;
  this->client = NULL;
  transport::IOManager::Instance()->Stop();
}

void Node::Init(const std::string &master_host, unsigned short master_port)
{
  std::cout << "Node init\n";
  this->client = new transport::Client(master_host, master_port);
}
