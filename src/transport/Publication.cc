#include "Publication.hh"

using namespace gazebo;
using namespace transport;

////////////////////////////////////////////////////////////////////////////////
// Constructor
Publication::Publication( const std::string &topic, const std::string &msgType )
  : topic(topic), msgType(msgType)
{
  this->prevMessage = "";
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Publication::~Publication()
{
}
        
////////////////////////////////////////////////////////////////////////////////
/// Get the topic for this publication
std::string Publication::GetTopic() const
{
  return this->topic;
}

////////////////////////////////////////////////////////////////////////////////
// Add a subscription callback
void Publication::AddSubscription(const CallbackHelperPtr &callback)
{
  this->callbacks.push_back(callback);

  // Send the previous message once a connection happens.
  if (!this->prevMessage.empty())
    callback->HandleMessage(this->prevMessage);
}

////////////////////////////////////////////////////////////////////////////////
/// Publish data
void Publication::Publish(const std::string &data)
{
  std::list< CallbackHelperPtr >::iterator iter;
  for (iter = this->callbacks.begin(); iter != this->callbacks.end(); iter++)
  {
    (*iter)->HandleMessage(data);
  }
  this->prevMessage = data;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the type of message
std::string Publication::GetMsgType() const
{
  return this->msgType;
}

void Publication::AddTransport( const PublicationTransportPtr &publink)
{
  publink->AddCallback( boost::bind(&Publication::Publish, this, _1) );
  this->transports.push_back( publink );
}

