#include "transport/TopicManager.hh"
#include "transport/ConnectionManager.hh"
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
  if (this->connection)
  {
    this->connection->DisconnectShutdownSignal(this->shutdownConnectionPtr);

    msgs::Subscribe sub;
    sub.set_topic(this->topic);
    sub.set_msg_type(this->msgType);
    sub.set_host( this->connection->GetLocalAddress() );
    sub.set_port( this->connection->GetLocalPort() );
    ConnectionManager::Instance()->Unsubscribe( sub );
    this->connection->StopRead();
    this->connection.reset();

    ConnectionManager::Instance()->RemoveConnection( this->connection );
  }
}

void PublicationTransport::Init(const ConnectionPtr &conn_)
{
  this->connection = conn_;
  msgs::Subscribe sub;
  sub.set_topic(this->topic);
  sub.set_msg_type(this->msgType);
  sub.set_host( this->connection->GetLocalAddress() );
  sub.set_port( this->connection->GetLocalPort() );

  this->connection->EnqueueMsg( msgs::Package("sub",sub) );
  // Put this in PublicationTransportPtr
  // Start reading messages from the remote publisher
  this->connection->StartRead(boost::bind(&PublicationTransport::OnPublish, this, _1));

  this->shutdownConnectionPtr = this->connection->ConnectToShutdownSignal(
      boost::bind(&PublicationTransport::OnConnectionShutdown, this));
}

void PublicationTransport::OnConnectionShutdown()
{
  gzdbg << "Publication transport connection shutdown\n";
}

void PublicationTransport::AddCallback(const boost::function<void(const std::string &)> &cb_)
{
  this->callback = cb_;
}

void PublicationTransport::OnPublish(const std::string &data)
{
  if (this->callback)
  {
    (this->callback)(data);
  }
}

const ConnectionPtr PublicationTransport::GetConnection() const
{
  return this->connection;
}
