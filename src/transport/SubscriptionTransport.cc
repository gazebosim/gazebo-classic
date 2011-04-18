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
void SubscriptionTransport::HandleMessage(const google::protobuf::Message *msg)
{
  std::string data;
  msg->SerializeToString(&data);
  this->connection->Write( data );
}

////////////////////////////////////////////////////////////////////////////////
/// Output a message to a connection
void SubscriptionTransport::HandleData(const std::string &newdata)
{
  this->connection->Write( newdata );
}
