/*
 * Copyright 2012 Open Source Robotics Foundation
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
#include <list>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <string>

#include "transport/Node.hh"
#include "transport/Publisher.hh"
#include "transport/Subscriber.hh"
#include "transport/ConnectionManager.hh"
#include "Transport.hh"

using namespace gazebo;

boost::thread *g_runThread = NULL;
boost::condition_variable g_responseCondition;
boost::mutex requestMutex;
bool g_stopped = true;

const msgs::Request *g_request = NULL;
msgs::Response *g_response = NULL;

/////////////////////////////////////////////////
bool transport::get_master_uri(std::string &master_host,
                               unsigned int &master_port)
{
  char *char_uri = getenv("GAZEBO_MASTER_URI");

  // Set to default host and port
  if (!char_uri || strlen(char_uri) == 0)
  {
    master_host = "localhost";
    master_port = 11345;
    return false;
  }

  std::string master_uri = char_uri;

  boost::replace_first(master_uri, "http://", "");
  int last_colon = master_uri.find_last_of(":");
  master_host = master_uri.substr(0, last_colon);
  master_port = boost::lexical_cast<unsigned int>(
      master_uri.substr(last_colon+1, master_uri.size() - (last_colon+1)));

  return true;
}

/////////////////////////////////////////////////
bool transport::init(const std::string &master_host, unsigned int master_port)
{
  std::string host = master_host;
  unsigned int port = master_port;

  if (host.empty())
    get_master_uri(host, port);

  transport::TopicManager::Instance()->Init();
  if (!transport::ConnectionManager::Instance()->Init(host, port))
    return false;

  return true;
}

/////////////////////////////////////////////////
void transport::run()
{
  g_stopped = false;
  g_runThread = new boost::thread(&transport::ConnectionManager::Run,
                                transport::ConnectionManager::Instance());

  std::list<std::string> namespaces;

  // This chunk of code just waits until we get a list of topic namespaces.
  unsigned int trys = 0;
  unsigned int limit = 50;
  while (namespaces.empty() && trys < limit)
  {
    TopicManager::Instance()->GetTopicNamespaces(namespaces);

    // 25 seconds max wait time
    common::Time::MSleep(500);

    trys++;
  }

  if (trys >= limit)
    gzerr << "Unable to get topic namespaces in [" << trys << "] tries.\n";
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
  transport::ConnectionManager::Instance()->Stop();
}

/////////////////////////////////////////////////
void transport::fini()
{
  g_stopped = true;
  transport::TopicManager::Instance()->Fini();
  transport::ConnectionManager::Instance()->Stop();

  if (g_runThread)
  {
    g_runThread->join();
    delete g_runThread;
    g_runThread = NULL;
  }
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
  if (!g_request || _msg->id() != g_request->id())
    return;

  if (!g_response)
    g_response = new msgs::Response;

  g_response->CopyFrom(*_msg);
  g_responseCondition.notify_one();
}

/////////////////////////////////////////////////
msgs::Response transport::request(const std::string &_worldName,
                                  const msgs::Request &_request)
{
  boost::unique_lock<boost::mutex> lock(requestMutex);
  g_response = NULL;
  g_request = &_request;

  NodePtr node = NodePtr(new Node());
  node->Init(_worldName);

  PublisherPtr requestPub = node->Advertise<msgs::Request>("~/request");
  SubscriberPtr responseSub = node->Subscribe("~/response", &on_response);

  requestPub->Publish(_request);

  g_responseCondition.wait(lock);

  requestPub.reset();
  responseSub.reset();
  node.reset();

  if (g_response != NULL)
    return *g_response;
  else
    return msgs::Response();
}

/////////////////////////////////////////////////
void transport::get_topic_namespaces(std::list<std::string> &_namespaces)
{
  TopicManager::Instance()->GetTopicNamespaces(_namespaces);
}
