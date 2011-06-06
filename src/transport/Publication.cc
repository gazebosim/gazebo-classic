#include "SubscriptionTransport.hh"
#include "Publication.hh"

using namespace gazebo;
using namespace transport;

////////////////////////////////////////////////////////////////////////////////
// Constructor
Publication::Publication( const std::string &topic, const std::string &msgType )
  : topic(topic), msgType(msgType)
{
  this->prevMsg = NULL;
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
  std::list< CallbackHelperPtr >::iterator iter;
  iter = std::find(this->callbacks.begin(), this->callbacks.end(), callback);
  if (iter == this->callbacks.end())
  {
    this->callbacks.push_back(callback);

    if (this->prevMsg && this->prevMsg->IsInitialized())
    {
      callback->HandleMessage(this->prevMsg);
    }
  } 
}

////////////////////////////////////////////////////////////////////////////////
// A a transport
void Publication::AddTransport( const PublicationTransportPtr &publink)
{
  publink->AddCallback( boost::bind(&Publication::PublishData, this, _1) );
  this->transports.push_back( publink );
}

void Publication::RemoveTransport(const std::string &host, unsigned int port)
{
  std::list<PublicationTransportPtr>::iterator iter;
  iter = this->transports.begin(); 
  while (iter != this->transports.end())
  {
    if ((*iter)->GetConnection()->GetRemoteAddress() == host &&
        (*iter)->GetConnection()->GetRemotePort() == port)
    {
      this->transports.erase( iter++ );
    }
    else 
      iter++;
  }
}

////////////////////////////////////////////////////////////////////////////////
void Publication::RemoveSubscription(const CallbackHelperPtr &callback)
{
  std::list< CallbackHelperPtr >::iterator iter;
  for (iter = this->callbacks.begin(); iter != this->callbacks.end(); iter++)
  {
    if (*iter == callback)
    {
      this->callbacks.erase(iter);
      break;
    }
  }

  // If no more subscribers, then disconnect from all publishers
  if (this->callbacks.size() == 0)
  {
    this->transports.clear();
  }
}

////////////////////////////////////////////////////////////////////////////////
// Remove a subscription
void Publication::RemoveSubscription(const std::string &host, unsigned int port)
{
  SubscriptionTransportPtr subptr;
  std::list< CallbackHelperPtr >::iterator iter;

  iter = this->callbacks.begin(); 
  while (iter != this->callbacks.end())
  {
    subptr = boost::shared_dynamic_cast<SubscriptionTransport>(*iter);
    if (subptr && subptr->GetConnection()->GetRemoteAddress() == host &&
        subptr->GetConnection()->GetRemotePort() == port)
    {
      this->callbacks.erase(iter++);
    }
    else
      iter++;
  }

  // If no more subscribers, then disconnect from all publishers
  if (this->callbacks.size() == 0)
  {
    this->transports.clear();
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Publish data
void Publication::Publish(const std::string &data)
{
  std::list< CallbackHelperPtr >::iterator iter;
  for (iter = this->callbacks.begin(); iter != this->callbacks.end(); iter++)
  {
    (*iter)->HandleData(data);
  }
}

////////////////////////////////////////////////////////////////////////////////
void Publication::PublishData(const std::string &data)
{
  this->Publish(data);
}

////////////////////////////////////////////////////////////////////////////////
void Publication::Publish(const google::protobuf::Message &msg,
                          const boost::function<void()> &cb)
{
  std::list< CallbackHelperPtr >::iterator iter;

  for (iter = this->callbacks.begin(); iter != this->callbacks.end(); iter++)
  {
    (*iter)->HandleMessage(&msg);
  }
  if (cb)
    (cb)();

  if (!this->prevMsg)
    this->prevMsg = msg.New();

  this->prevMsg->CopyFrom(msg);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the type of message
std::string Publication::GetMsgType() const
{
  return this->msgType;
}


