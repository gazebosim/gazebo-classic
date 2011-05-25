/*
 * Copyright 2011 Nate Koenig & Andrew Howard
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include "common/Messages.hh"
#include "transport/TopicManager.hh"
#include "transport/ConnectionManager.hh"

#include "gazebo_config.h"

using namespace gazebo;
using namespace transport;

////////////////////////////////////////////////////////////////////////////////
// Constructor
ConnectionManager::ConnectionManager()
  : masterConn( new Connection() ),
    serverConn( new Connection() )
{
  this->initialized = false;
  this->thread = NULL;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
ConnectionManager::~ConnectionManager()
{
  this->Stop();
  this->connections.clear();
  this->masterConn.reset();
  this->serverConn.reset();
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the connection manager
void ConnectionManager::Init(const std::string &master_host, 
                             unsigned short master_port)
{
  // Create a new TCP server on a free port
  this->serverConn->Listen(0, boost::bind(&ConnectionManager::OnAccept, this, _1) );

  this->masterConn->Connect(master_host, master_port);

  std::string initData;
  this->masterConn->Read(initData);
  msgs::Packet packet;
  packet.ParseFromString(initData);

  if (packet.type() == "init")
  {
    msgs::String msg;
    msg.ParseFromString( packet.serialized_data() );
    if (msg.data() == std::string("gazebo ") + GAZEBO_VERSION)
    {
      // TODO: set some flag.. maybe start "serverConn" when initialized
      gzmsg << "Connected to gazebo master @ " << this->masterConn->GetRemoteURI() << "\n";
    }
    else
    {
      // TODO: MAke this a proper error
      gzerr << "Conflicting gazebo versions\n";
    }
  }
  else
    gzerr << "Didn't receive an init from the master\n";

  this->masterConn->StartRead( boost::bind(&ConnectionManager::OnMasterRead, this, _1) );

  this->initialized = true;
  this->stop = false;
}

////////////////////////////////////////////////////////////////////////////////
// Finalize
void ConnectionManager::Fini()
{
  this->Stop();

  this->connections.clear();
  this->masterConn->Cancel();
  this->masterConn->StopRead();
  this->masterConn->Close();
}

////////////////////////////////////////////////////////////////////////////////
// Start the conneciton manager
void ConnectionManager::Start()
{
  this->stop = false;
  this->thread = new boost::thread( 
      boost::bind(&ConnectionManager::RunLoop, this));
}

////////////////////////////////////////////////////////////////////////////////
// Stop the conneciton manager
void ConnectionManager::Stop()
{
  this->stop = true;
  if (this->thread)
  {
    this->thread->join();
    delete this->thread;
    this->thread = NULL;
  }
}

////////////////////////////////////////////////////////////////////////////////
// Run all the connections
void ConnectionManager::RunLoop()
{
  std::list<ConnectionPtr>::iterator iter;
  while (!this->stop)
  {
    this->masterConn->ProcessWriteQueue();

    for (iter = this->connections.begin(); 
         iter != this->connections.end(); iter++)
    {
      (*iter)->ProcessWriteQueue();
    }
    usleep(10000);
  }
}

////////////////////////////////////////////////////////////////////////////////
// On read master
void ConnectionManager::OnMasterRead( const std::string &data)
{
  msgs::Packet packet;
  packet.ParseFromString(data);

  // Publisher_update. This occurs when we try to subscribe to a topic, and
  // the master informs us of a remote host that is publishing on our
  // requested topic
  if (packet.type() == "publisher_update")
  {
    msgs::Publish pub;
    pub.ParseFromString( packet.serialized_data() );

    // If we are connecting to a remote publisher, then make a proper
    // transport mechanism. Otherwise, the subscriber should have already
    // been locally connected in TopicManager::Advertise.
    if (pub.host() != this->serverConn->GetLocalAddress() ||
        pub.port() != this->serverConn->GetLocalPort())
    {
      // Connect to the remote publisher
      ConnectionPtr conn = this->ConnectToRemoteHost(pub.host(), pub.port());

      // Create a transport link that will read from the connection, and 
      // send data to a Publication.
      PublicationTransportPtr publink(new PublicationTransport(pub.topic(), 
                                      pub.msg_type()));
      publink->Init( conn );

      // Connect local subscribers to the publication transport link
      TopicManager::Instance()->ConnectSubToPub(pub.topic(), publink);
    }
  }
  else if (packet.type() == "unsubscribe")
  {
    msgs::Subscribe sub;
    sub.ParseFromString( packet.serialized_data() );

    // Disconnect a local publisher from a remote subscriber
    TopicManager::Instance()->DisconnectPubFromSub(sub.topic(), sub.host(), sub.port());
  }
  else if (packet.type() == "unadvertise")
  {
    msgs::Publish pub;
    pub.ParseFromString( packet.serialized_data() );

    // Disconnection all local subscribers from a remote publisher
    TopicManager::Instance()->DisconnectSubFromPub(pub.topic(), pub.host(), pub.port());
  }
  else
    gzerr << "ConnectionManager::OnMasterRead unknown type[" << packet.type() << "]\n";
}

////////////////////////////////////////////////////////////////////////////////
// On accept
void ConnectionManager::OnAccept(const ConnectionPtr &new_connection)
{
  new_connection->AsyncRead(
      boost::bind(&ConnectionManager::OnRead, this, new_connection, _1) );

  // Add the connection to the list of connections
  this->connections.push_back( new_connection );
}

////////////////////////////////////////////////////////////////////////////////
// On read header
void ConnectionManager::OnRead(const ConnectionPtr &connection, const std::string &data )
{
  msgs::Packet packet;
  packet.ParseFromString(data);

  // If we have an incoming (remote) subscription
  if (packet.type() == "sub")
  {
    msgs::Subscribe sub;
    sub.ParseFromString( packet.serialized_data() );

    // Create a transport link for the publisher to the remote subscriber
    // via the connection
    SubscriptionTransportPtr subLink( new SubscriptionTransport() );
    subLink->Init( connection );

    // Connect the publisher to this transport mechanism
    TopicManager::Instance()->ConnectPubToSub(sub.topic(), subLink);
  }
  else
    gzerr << "Error est here\n";
}

////////////////////////////////////////////////////////////////////////////////
// Advertise a topic
void ConnectionManager::Advertise(const std::string &topic, 
                                  const std::string &msgType)
{
  if (!this->initialized)
    return;

  msgs::Publish msg;
  msg.set_topic( topic );
  msg.set_msg_type( msgType );
  msg.set_host( this->serverConn->GetLocalAddress() );
  msg.set_port( this->serverConn->GetLocalPort() );

  this->masterConn->EnqueueMsg(common::Message::Package("advertise", msg));
}

////////////////////////////////////////////////////////////////////////////////
void ConnectionManager::Unadvertise( const std::string &topic )
{
  msgs::Publish msg;
  msg.set_topic( topic );
  msg.set_msg_type( "" );
  msg.set_host( this->serverConn->GetLocalAddress() );
  msg.set_port( this->serverConn->GetLocalPort() );

  this->masterConn->EnqueueMsg(common::Message::Package("unadvertise", msg));
}

////////////////////////////////////////////////////////////////////////////////
// Get all the publishers the master knows about
void ConnectionManager::GetAllPublishers(std::list<msgs::Publish> &publishers)
{
  std::string data;
  msgs::Request request;
  msgs::Packet packet;
  msgs::Publishers pubs;
  request.set_request("get_publishers");

  this->masterConn->StopRead();
  this->masterConn->Cancel();

  // Get the list of publishers
  this->masterConn->EnqueueMsg( common::Message::Package("request", request) );

  this->masterConn->Read(data);

  packet.ParseFromString( data );

  pubs.ParseFromString( packet.serialized_data() );
  for (int i=0; i < pubs.publisher_size(); i++)
  {
    const msgs::Publish &p = pubs.publisher(i);
    publishers.push_back(p);
  }

  this->masterConn->StartRead( 
      boost::bind(&ConnectionManager::OnMasterRead, this, _1) );
}

void ConnectionManager::Unsubscribe( const msgs::Subscribe &msg )
{
  // Inform the master that we want to unsubscribe from a topic.
  this->masterConn->EnqueueMsg(common::Message::Package("unsubscribe", msg));
}

void ConnectionManager::Subscribe(const std::string &topic, 
                                  const std::string &msgType)
{
  if (!this->initialized)
    return;

  // TODO:
  // Find a current connection on the topic
  //ConnectionPtr conn = this->FindConnection( topic );

  // If the connection to a remote publisher does not exist, then we need
  // to establish a connection.
  //if (!conn)
  {
    msgs::Subscribe msg;
    msg.set_topic( topic );
    msg.set_msg_type( msgType );
    msg.set_host( this->serverConn->GetLocalAddress() );
    msg.set_port( this->serverConn->GetLocalPort() );

    // Inform the master that we want to subscribe to a topic.
    // This will result in Connection::OnMasterRead getting called with a 
    // packet type of "publisher_update"
    this->masterConn->EnqueueMsg(common::Message::Package("subscribe", msg));
  }
}

////////////////////////////////////////////////////////////////////////////////
// Connect to a remote server
ConnectionPtr ConnectionManager::ConnectToRemoteHost( const std::string &host,
                                                       unsigned short port)
{
  ConnectionPtr conn;

  if (!this->initialized)
    return conn;

  // Connect to the remote host
  conn.reset(new Connection());
  conn->Connect(host, port);

  this->connections.push_back( conn );

  return conn;
}

////////////////////////////////////////////////////////////////////////////////
// Remove a connection
void ConnectionManager::RemoveConnection(ConnectionPtr &conn)
{
  std::list<ConnectionPtr>::iterator iter;

  iter = this->connections.begin(); 
  while (iter != this->connections.end())
  {
    if ( (*iter) == conn )
      this->connections.erase( iter++);
    else
      iter++;
  }
} 


////////////////////////////////////////////////////////////////////////////////
// Find a connection that matches a host and port
ConnectionPtr ConnectionManager::FindConnection(const std::string &host, 
                                                 unsigned short port)
{
  ConnectionPtr conn;

  std::list<ConnectionPtr>::iterator iter;

  // Check to see if we are already connected to the remote publisher
  for (iter = this->connections.begin(); 
       iter != this->connections.end(); iter++)
  {
    if ( (*iter)->GetRemoteAddress() == host && 
         (*iter)->GetRemotePort() == port)
      conn = *iter;
  }

  return conn;
}
