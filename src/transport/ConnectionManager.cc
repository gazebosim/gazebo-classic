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

#include "msgs/msgs.h"
#include "transport/TopicManager.hh"
#include "transport/ConnectionManager.hh"

#include "gazebo_config.h"

using namespace gazebo;
using namespace transport;

////////////////////////////////////////////////////////////////////////////////
// Constructor
ConnectionManager::ConnectionManager()
{
  this->tmpIndex = 0;
  this->initialized = false;
  this->stop = false;
  this->thread = NULL;

  this->listMutex = new boost::recursive_mutex();
  this->masterMessagesMutex = new boost::recursive_mutex();
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
ConnectionManager::~ConnectionManager()
{
  delete this->listMutex;
  this->listMutex = NULL;

  delete this->masterMessagesMutex;
  this->masterMessagesMutex = NULL;


  this->Fini();
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the connection manager
void ConnectionManager::Init(const std::string &master_host, 
                             unsigned short master_port)
{
  this->masterConn.reset( new Connection() );
  this->serverConn.reset( new Connection() );

  // Create a new TCP server on a free port
  this->serverConn->Listen(0, boost::bind(&ConnectionManager::OnAccept, this, _1) );

  this->masterConn->Connect(master_host, master_port);

  std::string initData, namespacesData, publishersData;
  this->masterConn->Read(initData);
  this->masterConn->Read(namespacesData);
  this->masterConn->Read(publishersData);

  
  msgs::Packet packet;
  packet.ParseFromString(initData);

  if (packet.type() == "version_init")
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

  packet.ParseFromString(namespacesData);
  if (packet.type() == "topic_namepaces_init")
  {
    msgs::String_V result;
    result.ParseFromString( packet.serialized_data() );
    this->listMutex->lock();
    for (int i=0; i < result.data_size(); i++)
    {
      this->namespaces.push_back( std::string(result.data(i)) );
    }
    this->listMutex->unlock();
  }
  else
    gzerr << "Did not get topic_namespaces_init msg from master\n";

  packet.ParseFromString(publishersData);
  if (packet.type() == "publishers_init")
  {
    msgs::Publishers pubs;
    pubs.ParseFromString( packet.serialized_data() );
    this->listMutex->lock();
    for (int i=0; i < pubs.publisher_size(); i++)
    {
      const msgs::Publish &p = pubs.publisher(i);
      this->publishers.push_back(p);
    }
    this->listMutex->unlock();
  }
  else
    gzerr << "Did not get publishers_init msg from master\n";

  /* DEBUG
  gzdbg << "Server URI[" << this->serverConn->GetLocalURI() << "]\n";
  gzdbg << "Master URI[" << this->masterConn->GetLocalURI() << "]\n";
  */

  this->masterConn->AsyncRead( 
      boost::bind(&ConnectionManager::OnMasterRead, this, _1));

  this->initialized = true;
  this->stop = false;
}

////////////////////////////////////////////////////////////////////////////////
// Finalize
void ConnectionManager::Fini()
{
  if (!this->initialized)
    return;

  this->masterConn->ProcessWriteQueue();
  this->masterConn.reset();

  this->serverConn->ProcessWriteQueue();
  this->serverConn.reset();

  this->connections.clear();
  this->initialized = false;

  this->Stop();
  this->initialized = false;
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

  if (this->masterConn)
  {
    this->masterConn->Shutdown();
  }
}

////////////////////////////////////////////////////////////////////////////////
// Run all the connections
void ConnectionManager::Run()
{
  std::list<ConnectionPtr>::iterator iter;
  this->stop = false;
  while (!this->stop)
  {

    this->masterMessagesMutex->lock();
    while (this->masterMessages.size() > 0)
    {
      this->ProcessMessage( this->masterMessages.front() );
      this->masterMessages.pop_front();
    }
    this->masterMessagesMutex->unlock();

    this->masterConn->ProcessWriteQueue();
    TopicManager::Instance()->ProcessNodes();

    iter = this->connections.begin();
    while (iter != this->connections.end())
    {
      if ((*iter)->IsOpen())
      {
        (*iter)->ProcessWriteQueue();
        iter++;
      }
      else
      {
        this->connections.erase( iter++);
      }
    }
    usleep(10000);
  }
}

////////////////////////////////////////////////////////////////////////////////
// On read master
void ConnectionManager::OnMasterRead( const std::string &_data)
{
  this->masterConn->AsyncRead( 
        boost::bind(&ConnectionManager::OnMasterRead, this, _1));

  if (!_data.empty())
  {
    this->masterMessagesMutex->lock();
    
    //DEBUG:printf("CM::OnMasterRead pushback message size[%d]\n",_data.size());

    this->masterMessages.push_back( std::string(_data) );
    this->masterMessagesMutex->unlock();
  }
  else
    gzerr << "ConnectionManager::OnMasterRead empty data\n";
}


void ConnectionManager::ProcessMessage(const std::string &_data)
{
  msgs::Packet packet;
  packet.ParseFromString(_data);

  //DEBUG: printf("CM::ProcessMessage size[%d]\n",_data.size());

  if (packet.type() == "publisher_add")
  {
    msgs::Publish result;
    result.ParseFromString( packet.serialized_data() );
    this->publishers.push_back(result);
  }
  else if (packet.type() == "publisher_del")
  {
    msgs::Publish result;
    result.ParseFromString( packet.serialized_data() );
    std::list<msgs::Publish>::iterator iter = this->publishers.begin();
    while (iter != this->publishers.end())
    {
      if ((*iter).topic() == result.topic() &&
          (*iter).host() == result.host() &&
          (*iter).port() == result.port())
        this->publishers.erase(iter);
      else
        iter++;
    }
  }
  else if (packet.type() == "topic_namespace_add")
  {
    msgs::String result;
    result.ParseFromString( packet.serialized_data() );
    this->listMutex->lock();
    this->namespaces.push_back( std::string(result.data()) );
    this->listMutex->unlock();
  }

  // Publisher_update. This occurs when we try to subscribe to a topic, and
  // the master informs us of a remote host that is publishing on our
  // requested topic
  else if (packet.type() == "publisher_update")
  {
    msgs::Publish pub;
    pub.ParseFromString( packet.serialized_data() );

    /* DEBUG
    if (pub.topic().find("scene") != std::string::npos)
      printf("ConnectionManager::OnMatsterRead Pub Update Topic[%s]\n",pub.topic().c_str());
      */

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
    TopicManager::Instance()->DisconnectSubFromPub(pub.topic(), 
        pub.host(), pub.port());
  }
  else
  {
    gzerr << "ConnectionManager::OnMasterRead unknown type[" 
          << packet.type() << "][" << packet.serialized_data() << "] Data[" << _data << "]\n";
  }
}

////////////////////////////////////////////////////////////////////////////////
// On accept
void ConnectionManager::OnAccept(const ConnectionPtr &newConnection)
{
  newConnection->AsyncRead( 
      boost::bind(&ConnectionManager::OnRead, this, newConnection, _1));

  // Add the connection to the list of connections
  this->connections.push_back( newConnection );
}

////////////////////////////////////////////////////////////////////////////////
// On read header
void ConnectionManager::OnRead(const ConnectionPtr &_connection, 
                               const std::string &_data )
{
  if (_data.empty())
  {
    gzerr << "Data was empty, try again\n";
    _connection->AsyncRead(
        boost::bind(&ConnectionManager::OnRead, this, _connection, _1));
    return;
  }

  msgs::Packet packet;
  packet.ParseFromString(_data);

  // If we have an incoming (remote) subscription
  if (packet.type() == "sub")
  {
    msgs::Subscribe sub;
    sub.ParseFromString( packet.serialized_data() );

    // Create a transport link for the publisher to the remote subscriber
    // via the connection
    SubscriptionTransportPtr subLink( new SubscriptionTransport() );
    subLink->Init( _connection );

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

  this->masterConn->EnqueueMsg(msgs::Package("advertise", msg));
}

////////////////////////////////////////////////////////////////////////////////
void ConnectionManager::RegisterTopicNamespace(const std::string &_name)
{
  if (!this->initialized)
    return;

  msgs::String msg;
  msg.set_data( _name );
  this->masterConn->EnqueueMsg(msgs::Package("register_topic_namespace", msg));
}

////////////////////////////////////////////////////////////////////////////////
void ConnectionManager::Unadvertise( const std::string &topic )
{
  msgs::Publish msg;
  msg.set_topic( topic );
  msg.set_msg_type( "" );

  if (this->serverConn)
  {
    msg.set_host( this->serverConn->GetLocalAddress() );
    msg.set_port( this->serverConn->GetLocalPort() );
  }

  if (this->masterConn)
  {
    this->masterConn->EnqueueMsg(msgs::Package("unadvertise", msg), true);
  }
}

////////////////////////////////////////////////////////////////////////////////
void ConnectionManager::GetAllPublishers(std::list<msgs::Publish> &_publishers)
{
  _publishers.clear();
  std::list<msgs::Publish>::iterator iter;

  this->listMutex->lock();
  for (iter = this->publishers.begin(); iter != this->publishers.end(); iter++)
    _publishers.push_back(*iter);
  this->listMutex->unlock();
}

////////////////////////////////////////////////////////////////////////////////
void ConnectionManager::GetTopicNamespaces( std::list<std::string> &_namespaces)
{
  _namespaces.clear();
  std::list<std::string>::iterator iter;

  this->listMutex->lock();
  for (iter = this->namespaces.begin(); iter != this->namespaces.end(); iter++)
    _namespaces.push_back(*iter);
  this->listMutex->unlock();
}

void ConnectionManager::Unsubscribe( const msgs::Subscribe &_sub )
{
  // Inform the master that we want to unsubscribe from a topic.
  this->masterConn->EnqueueMsg(msgs::Package("unsubscribe", _sub), true);
}

void ConnectionManager::Unsubscribe( const std::string &_topic,
                                     const std::string &_msgType )
{
  msgs::Subscribe msg;
  msg.set_topic( _topic );
  msg.set_msg_type( _msgType );
  msg.set_host( this->serverConn->GetLocalAddress() );
  msg.set_port( this->serverConn->GetLocalPort() );

  // Inform the master that we want to unsubscribe from a topic.
  this->masterConn->EnqueueMsg(msgs::Package("unsubscribe", msg), true);
}

void ConnectionManager::Subscribe(const std::string &topic, 
                                  const std::string &msgType)
{
  if (!this->initialized)
  {
    gzerr << "ConnectionManager is not initialized\n";
    return;
  }

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
    this->masterConn->EnqueueMsg(msgs::Package("subscribe", msg));
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

  conn = this->FindConnection(host,port);
  if (!conn)
  {
    // Connect to the remote host
    conn.reset(new Connection());
    conn->Connect(host, port);

    this->connections.push_back( conn );
  }

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

  std::string uri = "http://" + host + ":" + boost::lexical_cast<std::string>(port);

  // Check to see if we are already connected to the remote publisher
  for (iter = this->connections.begin(); 
       iter != this->connections.end(); iter++)
  {
    if ( (*iter)->GetRemoteURI() == uri)
      conn = *iter;
  }

  return conn;
}
