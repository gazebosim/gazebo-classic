#include "SubscriptionTransport.hh"
#include "Publication.hh"

using namespace gazebo;
using namespace transport;

unsigned int Publication::idCounter = 0;

////////////////////////////////////////////////////////////////////////////////
// Constructor
Publication::Publication( const std::string &topic, const std::string &msgType )
  : topic(topic), msgType(msgType), locallyAdvertised(false)
{
  this->id = idCounter++;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Publication::~Publication()
{
  std::list<google::protobuf::Message*>::iterator iter;
  for (iter = this->prevMsgBuffer.begin();
       iter != this->prevMsgBuffer.end(); iter++)
  {
    delete (*iter);
  }
  this->prevMsgBuffer.clear();
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

    std::list<google::protobuf::Message*>::iterator msgIter;
    for (msgIter = this->prevMsgBuffer.begin();
         msgIter != this->prevMsgBuffer.end(); msgIter++)
    {
      if ((*msgIter)->IsInitialized())
      {
        callback->HandleMessage(*msgIter);
      }
    }
  } 
}

////////////////////////////////////////////////////////////////////////////////
// A a transport
void Publication::AddTransport( const PublicationTransportPtr &_publink)
{
  bool add = true;

  // Find an existing publication transport
  std::list<PublicationTransportPtr>::iterator iter;
  for (iter = this->transports.begin(); iter != this->transports.end(); iter++)
  {
    if ((*iter)->GetTopic() == _publink->GetTopic() &&
        (*iter)->GetMsgType() == _publink->GetMsgType() &&
        (*iter)->GetConnection()->GetRemoteURI() ==
        _publink->GetConnection()->GetRemoteURI())
    {
      add = false;
      break;
    }
  }

  // Don't add a duplicate transport
  if (add)
  {
    _publink->AddCallback( boost::bind(&Publication::LocalPublish, this, _1) );
    this->transports.push_back( _publink );
  }
}

bool Publication::HasTransport( const std::string &_host, unsigned int _port )
{
  std::list<PublicationTransportPtr>::iterator iter;
  for (iter = this->transports.begin(); iter != this->transports.end(); iter++)
  {
    if ( (*iter)->GetConnection()->GetRemoteAddress() == _host &&
         (*iter)->GetConnection()->GetRemotePort() == _port)
    {
      return true;
    }
  }

  return false;
}

////////////////////////////////////////////////////////////////////////////////
// Remove a transport
void Publication::RemoveTransport(const std::string &host_, unsigned int port_)
{
  std::list<PublicationTransportPtr>::iterator iter;
  iter = this->transports.begin(); 
  while (iter != this->transports.end())
  {
    if (!(*iter)->GetConnection()->IsOpen() || 
        ((*iter)->GetConnection()->GetRemoteAddress() == host_ &&
         (*iter)->GetConnection()->GetRemotePort() == port_) )
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
    if (!subptr || !subptr->GetConnection()->IsOpen() ||
        (subptr->GetConnection()->GetRemoteAddress() == host &&
         subptr->GetConnection()->GetRemotePort() == port))
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
  iter = this->callbacks.begin();

  while (iter != this->callbacks.end())
  {
    if ((*iter)->HandleData(data))
      iter++;
    else
    {
      this->callbacks.erase( iter++ );
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
// Publish data only on local subscriptions
void Publication::LocalPublish(const std::string &data)
{
  std::list< CallbackHelperPtr >::iterator iter;
  iter = this->callbacks.begin();

  while (iter != this->callbacks.end())
  {
    if ((*iter)->IsLocal())
    {
      if ((*iter)->HandleData(data))
        iter++;
      else
        iter = this->callbacks.erase( iter );
    }
    else
      iter++;
  }

}

////////////////////////////////////////////////////////////////////////////////
void Publication::Publish(const google::protobuf::Message &_msg,
                          const boost::function<void()> &_cb)
{
  std::list< CallbackHelperPtr >::iterator iter;

  std::string data;
  _msg.SerializeToString(&data);

  iter = this->callbacks.begin();
  while (iter != this->callbacks.end())
  {
    if ((*iter)->HandleData(data))
    {
      iter++;
    }
    else
    {
      this->callbacks.erase( iter++ );
    }
  }
  if (_cb)
    (_cb)();

  if (this->prevMsgBuffer.size() > 10)
  {
    delete this->prevMsgBuffer.front();
    this->prevMsgBuffer.pop_front();
  }
  this->prevMsgBuffer.push_back(_msg.New());
  this->prevMsgBuffer.back()->CopyFrom(_msg);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the type of message
std::string Publication::GetMsgType() const
{
  return this->msgType;
}


unsigned int Publication::GetTransportCount()
{
  return this->transports.size();
}

unsigned int Publication::GetCallbackCount()
{
  return this->callbacks.size();
}

////////////////////////////////////////////////////////////////////////////////
unsigned int Publication::GetRemoteSubscriptionCount()
{
  std::list< CallbackHelperPtr >::iterator iter;
  unsigned int count = 0;

  for (iter = this->callbacks.begin(); iter != this->callbacks.end(); iter++)
  {
    if ( !(*iter)->IsLocal() )
      count++;
  }

  return count;
}

////////////////////////////////////////////////////////////////////////////////
/// Return true if the topic has been advertised from this process.
bool Publication::GetLocallyAdvertised() const
{
  return this->locallyAdvertised;
}

////////////////////////////////////////////////////////////////////////////////
/// Set whether this topic has been advertised from this process
void Publication::SetLocallyAdvertised(bool _value)
{
  this->locallyAdvertised = _value;
}
