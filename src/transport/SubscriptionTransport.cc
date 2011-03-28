#include "SubscriptionTransport.hh"

using namespace gazebo;
using namespace transport;

////////////////////////////////////////////////////////////////////////////////
// Constructor
SubscriptionTransport::SubscriptionTransport()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
SubscriptionTransport::~SubscriptionTransport()
{
}

////////////////////////////////////////////////////////////////////////////////
/// Initialize the publication link 
void SubscriptionTransport::Init( const ConnectionPtr &conn )
{
  this->connection = conn;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the typename of the message that is handled
std::string SubscriptionTransport::GetMsgType() const
{
  return "";
}

////////////////////////////////////////////////////////////////////////////////
/// Output a message to a connection
void SubscriptionTransport::HandleMessage(const std::string &newdata)
{
  this->connection->Write( newdata );
}
