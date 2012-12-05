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

#include <google/protobuf/message.h>

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include "common/Time.hh"

#include "transport/Transport.hh"
#include "transport/TransportTypes.hh"
#include "transport/Node.hh"

#include "gazebo_config.h"

using namespace gazebo;

// transport::ConnectionPtr connection(new transport::Connection());
std::vector<std::string> params;

common::Time hz_prev_time;

/////////////////////////////////////////////////
void help()
{
  std::cerr << "This tool lists information about published topics on a "
            << "Gazebo master.\n"
            << "    list         : List all topics\n"
            << "    info <topic> : Get information about a topic\n"
            << "    echo <topic> : Output topic data to screen\n"
            << "    hz <topic>   : Get publish frequency\n"
            << "    help         : This help text\n";
}

/////////////////////////////////////////////////
bool parse(int argc, char **argv)
{
  if (argc == 1 || std::string(argv[1]) == "help")
  {
    help();
    return false;
  }

  // Get parameters from command line
  for (int i = 1; i < argc; i++)
  {
    std::string p = argv[i];
    boost::trim(p);
    params.push_back(p);
  }

  // Get parameters from stdin
  if (!isatty(fileno(stdin)))
  {
    char str[1024];
    while (!feof(stdin))
    {
      if (fgets(str, 1024, stdin)== NULL)
        break;

      if (feof(stdin))
        break;
      std::string p = str;
      boost::trim(p);
      params.push_back(p);
    }
  }

  return true;
}

/////////////////////////////////////////////////
transport::ConnectionPtr connect_to_master(const std::string &host,
                                           unsigned short port)
{
  std::string data, namespacesData, publishersData;
  msgs::Packet packet;

  // Connect to the master
  transport::ConnectionPtr connection(new transport::Connection());
  connection->Connect(host, port);

  // Read the verification message
  connection->Read(data);
  connection->Read(namespacesData);
  connection->Read(publishersData);

  packet.ParseFromString(data);
  if (packet.type() == "init")
  {
    msgs::GzString msg;
    msg.ParseFromString(packet.serialized_data());
    if (msg.data() != std::string("gazebo ") + GAZEBO_VERSION_FULL)
      std::cerr << "Conflicting gazebo versions\n";
  }

  return connection;
}

/////////////////////////////////////////////////
void list()
{
  std::string data;
  msgs::Packet packet;
  msgs::Request request;
  msgs::Publishers pubs;

  transport::ConnectionPtr connection = connect_to_master("localhost", 11345);

  request.set_id(0);
  request.set_request("get_publishers");
  connection->EnqueueMsg(msgs::Package("request", request), true);
  connection->Read(data);

  packet.ParseFromString(data);
  pubs.ParseFromString(packet.serialized_data());

  for (int i = 0; i < pubs.publisher_size(); i++)
  {
    const msgs::Publish &p = pubs.publisher(i);
    if (p.topic().find("__dbg") == std::string::npos)
      std::cout << p.topic() << std::endl;
  }

  connection.reset();
}

/////////////////////////////////////////////////
void echo_cb(ConstGzStringPtr &_data)
{
  std::cout << _data->data() << "\n";
}

/////////////////////////////////////////////////
void hz_cb(ConstGzStringPtr &/*_data*/)
{
  common::Time cur_time = common::Time::GetWallTime();

  if (hz_prev_time != common::Time(0, 0))
    printf("Hz: %6.2f\n", 1.0 / (cur_time - hz_prev_time).Double());

  hz_prev_time = cur_time;
}

/////////////////////////////////////////////////
msgs::TopicInfo get_topic_info(const std::string &_topic)
{
  msgs::TopicInfo topic_info;
  std::string data;
  msgs::Request *request = msgs::CreateRequest("topic_info", _topic);
  msgs::Packet packet;

  transport::ConnectionPtr connection = connect_to_master("localhost", 11345);

  connection->EnqueueMsg(msgs::Package("request", *request), true);

  int i = 0;
  do
  {
    connection->Read(data);
    packet.ParseFromString(data);
  } while (packet.type() != "topic_info_response" && ++i < 10);

  if (i <10)
    topic_info.ParseFromString(packet.serialized_data());
  else
    std::cerr << "Unable to get topic info.\n";

  delete request;
  return topic_info;
}

/////////////////////////////////////////////////
void print_topic_info(const std::string &_topic)
{
  msgs::TopicInfo info = get_topic_info(_topic);
  std::cout << "Type: " << info.msg_type() << "\n\n";

  std::cout << "Publishers:\n";
  for (int i = 0; i < info.publisher_size(); i++)
  {
    std::cout << "\t" << info.publisher(i).host() << ":"
              << info.publisher(i).port() << "\n";
  }

  std::cout << "\nSubscribers:\n";
  for (int i = 0; i < info.subscriber_size(); i++)
  {
    std::cout << "\t" << info.subscriber(i).host() << ":"
              << info.subscriber(i).port() << "\n";
  }
  std::cout << "\n";
}

/////////////////////////////////////////////////
void echo()
{
  if (params[1].empty())
  {
    std::cerr << "Error: No topic specified.\n";
    return;
  }

  transport::init();

  transport::NodePtr node(new transport::Node());
  node->Init();

  std::string topic = params[1];
  topic +=  "/__dbg";

  transport::SubscriberPtr sub = node->Subscribe(topic, echo_cb);

  // Run the transport loop: starts a new thread
  transport::run();

  while (true)
    common::Time::MSleep(10);

  transport::fini();
}

/////////////////////////////////////////////////
void hz()
{
  if (params[1].empty())
  {
    std::cerr << "Error: No topic specified.\n";
    return;
  }

  transport::init();

  transport::NodePtr node(new transport::Node());
  node->Init();

  std::string topic = params[1];
  topic +=  "/__dbg";

  transport::SubscriberPtr sub = node->Subscribe(topic, hz_cb);

  // Run the transport loop: starts a new thread
  transport::run();

  while (true)
    common::Time::MSleep(10);

  transport::fini();
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  if (!parse(argc, argv))
    return 0;

  if (params[0] == "list")
    list();
  else if (params[0] == "info")
    print_topic_info(params[1]);
  else if (params[0] == "echo")
    echo();
  else if (params[0] == "hz")
    hz();
}
