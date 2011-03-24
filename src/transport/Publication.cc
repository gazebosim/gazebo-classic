#include "Publication.hh"

using namespace gazebo;
using namespace transport;

////////////////////////////////////////////////////////////////////////////////
// Constructor
Publication::Publication( const std::string &topic )
  : topic(topic)
{
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
}

////////////////////////////////////////////////////////////////////////////////
/// Publish data
void Publication::Publish(const std::string &data)
{
  std::list< CallbackHelperPtr >::iterator iter;
  std::cout << "publication publish[" << this->callbacks.size() << "]\n";
  for (iter = this->callbacks.begin(); iter != this->callbacks.end(); iter++)
  {
    (*iter)->HandleMessage(data);
  }
}

