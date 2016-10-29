/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include <list>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <string>

#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Publisher.hh"
#include "gazebo/transport/Subscriber.hh"
#include "gazebo/transport/ConnectionManager.hh"
#include "gazebo/transport/TransportIface.hh"

using namespace gazebo;

boost::thread *g_runThread = NULL;
boost::condition_variable g_responseCondition;
boost::mutex requestMutex;
bool g_stopped = true;
bool g_minimalComms = false;

std::list<msgs::Request *> g_requests;
std::list<boost::shared_ptr<msgs::Response> > g_responses;

/////////////////////////////////////////////////
void dummy_callback_fn(uint32_t)
{
}

/////////////////////////////////////////////////
bool transport::get_master_uri(std::string &_masterHost,
                               unsigned int &_masterPort)
{
  char *charURI = getenv("GAZEBO_MASTER_URI");

  // Set to default host and port
  if (!charURI || strlen(charURI) == 0)
  {
    _masterHost = GAZEBO_DEFAULT_MASTER_HOST;
    _masterPort = GAZEBO_DEFAULT_MASTER_PORT;
    return false;
  }

  std::string masterURI = charURI;

  boost::replace_first(masterURI, "http://", "");
  size_t lastColon = masterURI.find_last_of(":");
  _masterHost = masterURI.substr(0, lastColon);

  if (lastColon == std::string::npos)
  {
    gzerr << "Port missing in master URI[" << masterURI
          << "]. Using default value of " << GAZEBO_DEFAULT_MASTER_PORT
          << ".\n";
    _masterPort = GAZEBO_DEFAULT_MASTER_PORT;
  }
  else
  {
    try
    {
      _masterPort = boost::lexical_cast<unsigned int>(
          masterURI.substr(lastColon + 1, masterURI.size() - (lastColon + 1)));
    }
    catch(...)
    {
      gzerr << "Unable to port from GAZEBO_MASTER_URI[" << masterURI << "]."
        << "Using the default port number of 11345.\n";
    }
  }

  return true;
}

/////////////////////////////////////////////////
bool transport::init(const std::string &_masterHost, unsigned int _masterPort,
    uint32_t _timeoutIterations)
{
  std::string host = _masterHost;
  unsigned int port = _masterPort;

  if (host.empty())
    get_master_uri(host, port);

  transport::TopicManager::Instance()->Init();

  if (!transport::ConnectionManager::Instance()->Init(host, port,
        _timeoutIterations))
    return false;

  return true;
}

/////////////////////////////////////////////////
void transport::run()
{
  g_stopped = false;
  g_runThread = new boost::thread(&transport::ConnectionManager::Run,
                                transport::ConnectionManager::Instance());
}

/////////////////////////////////////////////////
bool transport::is_stopped()
{
  return g_stopped;
}

/////////////////////////////////////////////////
void transport::stop()
{
  g_stopped = true;
  g_responseCondition.notify_all();

  transport::ConnectionManager::Instance()->Stop();
}

/////////////////////////////////////////////////
void transport::fini()
{
  transport::stop();
  if (g_runThread)
  {
    g_runThread->join();
    delete g_runThread;
    g_runThread = NULL;
  }
  transport::TopicManager::Instance()->Fini();
  transport::ConnectionManager::Instance()->Fini();
}

/////////////////////////////////////////////////
void transport::clear_buffers()
{
  transport::TopicManager::Instance()->ClearBuffers();
}

/////////////////////////////////////////////////
void transport::pause_incoming(bool _pause)
{
  transport::TopicManager::Instance()->PauseIncoming(_pause);
}

/////////////////////////////////////////////////
void on_response(ConstResponsePtr &_msg)
{
  if (g_requests.empty() || g_stopped)
    return;

  std::list<msgs::Request *>::iterator iter;
  for (iter = g_requests.begin(); iter != g_requests.end(); ++iter)
  {
    if (_msg->id() == (*iter)->id())
      break;
  }

  // Stop if the response is not for any of the request messages.
  if (iter == g_requests.end())
    return;

  boost::mutex::scoped_lock lock(requestMutex);
  boost::shared_ptr<msgs::Response> response(new msgs::Response);
  response->CopyFrom(*_msg);
  g_responses.push_back(response);

  g_responseCondition.notify_all();
}

/////////////////////////////////////////////////
void transport::get_topic_namespaces(std::list<std::string> &_namespaces)
{
  TopicManager::Instance()->GetTopicNamespaces(_namespaces);
}

/////////////////////////////////////////////////
boost::shared_ptr<msgs::Response> transport::request(
    const std::string &_worldName, const std::string &_request,
    const std::string &_data)
{
  msgs::Request *request = msgs::CreateRequest(_request, _data);

  g_requests.push_back(request);

  NodePtr node = NodePtr(new Node());
  node->Init(_worldName);

  SubscriberPtr responseSub = node->Subscribe("~/response", &on_response);

  PublisherPtr requestPub = node->Advertise<msgs::Request>("~/request");
  requestPub->WaitForConnection();

  boost::mutex::scoped_lock lock(requestMutex);
  requestPub->Publish(*request, true);

  boost::shared_ptr<msgs::Response> response;
  std::list<boost::shared_ptr<msgs::Response> >::iterator iter;

  bool valid = false;
  while (!valid && !g_stopped)
  {
    // Wait for a response
    g_responseCondition.wait(lock);

    for (iter = g_responses.begin(); iter != g_responses.end(); ++iter)
    {
      if ((*iter)->id() == request->id())
      {
        response = *iter;
        g_responses.erase(iter);
        valid = true;
        break;
      }
    }
  }

  node->Fini();
  requestPub.reset();
  responseSub.reset();
  node.reset();

  delete request;
  return response;
}

/////////////////////////////////////////////////
void transport::requestNoReply(const std::string &_worldName,
                               const std::string &_request,
                               const std::string &_data)
{
  // Create a node for communication.
  NodePtr node = NodePtr(new Node());

  // Initialize the node, use the world name for the topic namespace.
  node->Init(_worldName);

  // Process the request.
  requestNoReply(node, _request, _data);

  // Cleanup the node.
  node.reset();
}

/////////////////////////////////////////////////
void transport::requestNoReply(NodePtr _node, const std::string &_request,
                               const std::string &_data)
{
  // Create a publisher on the request topic.
  PublisherPtr requestPub = _node->Advertise<msgs::Request>("~/request");

  // Create a new request message
  msgs::Request *request = msgs::CreateRequest(_request, _data);

  // Publish the request message
  requestPub->Publish(*request);

  // Cleanup the request
  delete request;

  // Clean up the publisher.
  requestPub.reset();
}

/////////////////////////////////////////////////
std::map<std::string, std::list<std::string> > transport::getAdvertisedTopics()
{
  std::map<std::string, std::list<std::string> > result;
  std::list<msgs::Publish> publishers;

  ConnectionManager::Instance()->GetAllPublishers(publishers);

  for (std::list<msgs::Publish>::iterator iter = publishers.begin();
      iter != publishers.end(); ++iter)
  {
    result[(*iter).msg_type()].push_back((*iter).topic());
  }

  return result;
}

/////////////////////////////////////////////////
std::list<std::string> transport::getAdvertisedTopics(
    const std::string &_msgType)
{
  std::list<std::string> result;
  std::list<msgs::Publish> publishers;

  ConnectionManager::Instance()->GetAllPublishers(publishers);

  for (std::list<msgs::Publish>::iterator iter = publishers.begin();
      iter != publishers.end(); ++iter)
  {
    if (std::find(result.begin(), result.end(), (*iter).topic()) !=
        result.end())
      continue;

    if (_msgType.empty() || _msgType == (*iter).msg_type())
      result.push_back((*iter).topic());
  }

  return result;
}

/////////////////////////////////////////////////
std::string transport::getTopicMsgType(const std::string &_topicName)
{
  std::string result;
  std::list<msgs::Publish> publishers;

  ConnectionManager::Instance()->GetAllPublishers(publishers);

  for (std::list<msgs::Publish>::iterator iter = publishers.begin();
      iter != publishers.end() && result.empty(); ++iter)
  {
    if (_topicName == (*iter).topic())
      result = (*iter).msg_type();
  }

  return result;
}

/////////////////////////////////////////////////
void transport::setMinimalComms(bool _enabled)
{
  g_minimalComms = _enabled;
}

/////////////////////////////////////////////////
bool transport::getMinimalComms()
{
  return g_minimalComms;
}

/////////////////////////////////////////////////
transport::ConnectionPtr transport::connectToMaster()
{
  std::string data, namespacesData, publishersData;
  msgs::Packet packet;

  std::string host;
  unsigned int port;
  transport::get_master_uri(host, port);

  // Connect to the master
  transport::ConnectionPtr connection(new transport::Connection());

  if (connection->Connect(host, port))
  {
    try
    {
      // Read the verification message
      connection->Read(data);
      connection->Read(namespacesData);
      connection->Read(publishersData);
    }
    catch(...)
    {
      gzerr << "Unable to read from master\n";
      return transport::ConnectionPtr();
    }

    packet.ParseFromString(data);
    if (packet.type() == "version_init")
    {
      msgs::GzString msg;
      msg.ParseFromString(packet.serialized_data());
      if (msg.data() != std::string("gazebo ") + GAZEBO_VERSION)
        std::cerr << "Conflicting gazebo versions\n";
    }
    else
    {
      gzerr  << "Failed to received version_init packet from master. "
        << "Connection to master failed\n";
      connection.reset();
    }
  }
  else
  {
    gzerr << "Unable to connect to master.\n";
    connection.reset();
  }

  if (connection && !connection->IsOpen())
  {
    gzerr << "Connection to master instatiated, but socket is not open."
      << "Connection to master failed\n";
    connection.reset();
  }

  return connection;
}

/////////////////////////////////////////////////
bool transport::waitForNamespaces(const gazebo::common::Time &_maxWait)
{
  std::list<std::string> namespaces;
  gazebo::common::Time startTime = gazebo::common::Time::GetWallTime();

  gazebo::common::Time waitTime = std::min(
      gazebo::common::Time(0, 100000000), _maxWait / 10);

  gazebo::transport::TopicManager::Instance()->GetTopicNamespaces(
      namespaces);

  while (namespaces.empty() &&
      gazebo::common::Time::GetWallTime() - startTime < _maxWait)
  {
    gazebo::transport::TopicManager::Instance()->GetTopicNamespaces(
        namespaces);
    gazebo::common::Time::Sleep(waitTime);
  }

  if (gazebo::common::Time::GetWallTime() - startTime <= _maxWait)
    return true;
  return false;
}
