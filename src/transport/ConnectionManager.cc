#include "ConnectionManager.hh"

using namespace gazebo;
using namespace transport;

ConnectionManager::ConnectionManager()
{
}

ConnectionManager::~ConnectionManager()
{
}

void ConnectionManager::Run()
{
  this->connection = ConnectionPtr(new Connection());
}
