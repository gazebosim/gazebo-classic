#include "transport/TopicManager.hh"
#include "transport/PublicationTransport.hh"

using namespace gazebo;
using namespace transport;

PublicationTransport::PublicationTransport(const std::string &topic, 
                                           const std::string &msgType)
 : topic(topic), msgType(msgType)
{
  TopicManager::Instance()->UpdatePublications(topic, msgType);
}

PublicationTransport::~PublicationTransport()
{
  //TODO: Unadvertise 
}

void PublicationTransport::Init(const ConnectionPtr &conn)
{
  msgs::Subscribe sub;
  sub.set_topic(this->topic);
  sub.set_msg_type(this->msgType);
  sub.set_host( conn->GetLocalAddress() );
  sub.set_port( conn->GetLocalPort() );

  conn->Write( common::Message::Package("sub",sub) );
  // Put this in PublicationTransportPtr
  // Start reading messages from the remote publisher
  conn->StartRead(boost::bind(&PublicationTransport::OnPublish, this, _1));
}

void PublicationTransport::AddCallback(const boost::function<void(const std::string &)> &cb)
{
  this->callback = cb;
}

void PublicationTransport::OnPublish(const std::string &data)
{
  if (this->callback)
  {
    (this->callback)(data);
  }
}
