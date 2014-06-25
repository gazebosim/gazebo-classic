/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

#include <gazebo/gui/qt.h>
#include <gazebo/gui/TopicSelector.hh>
#include <gazebo/gui/viewers/TopicView.hh>
#include <gazebo/gui/viewers/ViewFactory.hh>
#include <gazebo/gazebo.hh>

#include <gazebo/common/Time.hh>
#include <gazebo/transport/TransportIface.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/transport/Node.hh>

#include <gazebo/gazebo_config.h>

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/program_options.hpp>
#include <boost/thread/mutex.hpp>

namespace po = boost::program_options;

using namespace gazebo;

// transport::ConnectionPtr connection(new transport::Connection());
std::vector<std::string> params;

common::Time hz_prev_time;
common::Time bw_prev_time;

std::vector<int> bwBytes;
std::vector<common::Time> bwTime;

boost::mutex mutex;

boost::shared_ptr<google::protobuf::Message> g_echoMsg;
bool g_useShortDebugString = false;

/////////////////////////////////////////////////
void help(po::options_description &_options)
{
  std::cerr << "gztopic -- DEPRECATED(see 'gz help topic')\n\n";

  std::cerr << "`gztopic` [options] <command>\n\n";

  std::cerr << "List information about published topics on a "
    "Gazebo master.\n\n";

  std::cerr << "Commands:\n"
            << "    list          List all topics.\n"
            << "    info <topic>  Get information about a topic.\n"
            << "    echo <topic>  Output topic data to screen.\n"
            << "    view <topic>  View topic data using a QT widget.\n"
            << "    hz <topic>    Get publish frequency.\n"
            << "    bw <topic>    Get topic bandwidth.\n"
            << "    help          This help text.\n\n";

  std::cerr << _options << "\n";

  std::cerr << "See also:\n"
    << "Examples and more information can be found at:"
    << "http://gazebosim.org/wiki/Tools#Topic_Info\n";
}

/////////////////////////////////////////////////
bool parse(int argc, char **argv)
{
  // Hidden options
  po::options_description hiddenOptions("hidden options");
  hiddenOptions.add_options()
    ("command", po::value<std::string>(), "Command")
    ("topic", po::value<std::string>(), "Topic");

  // Options that are visible to the user through help.
  po::options_description visibleOptions("Options");
  visibleOptions.add_options()
    ("help,h", "Output this help message.")
    ("unformatted,u", "Output the data from echo and list without formatting.");

  // Both the hidden and visible options
  po::options_description allOptions("all options");
  allOptions.add(hiddenOptions).add(visibleOptions);

  // The command and file options are positional
  po::positional_options_description positional;
  positional.add("command", 1).add("topic", -1);

  po::variables_map vm;

  try
  {
    po::store(
        po::command_line_parser(argc, argv).options(allOptions).positional(
          positional).run(), vm);

    po::notify(vm);
  }
  catch(boost::exception &_e)
  {
    std::cerr << "Invalid arguments\n\n";
    return false;
  }

  {
    std::string command;
    command = vm.count("command") ? vm["command"].as<std::string>() : "";

    if (command.empty() || command == "help" || vm.count("help"))
    {
      help(visibleOptions);
      return false;
    }

    // Get parameters from command line
    if (!command.empty())
      params.push_back(command);

    if (vm.count("unformatted"))
      g_useShortDebugString = true;
  }

  {
    std::string topic;
    topic = vm.count("topic") ? vm["topic"].as<std::string>() : "";
    if (!topic.empty())
      params.push_back(topic);
  }

  for (int i = 1; i < argc; i++)
  {
    std::string p = argv[i];
    boost::trim(p);
    params.push_back(p);
  }

  return true;
}

/////////////////////////////////////////////////
transport::ConnectionPtr connect_to_master()
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
    if (packet.type() == "init")
    {
      msgs::GzString msg;
      msg.ParseFromString(packet.serialized_data());
      if (msg.data() != std::string("gazebo ") + GAZEBO_VERSION_FULL)
        std::cerr << "Conflicting gazebo versions\n";
    }
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

  transport::ConnectionPtr connection = connect_to_master();

  if (connection)
  {
    request.set_id(0);
    request.set_request("get_publishers");
    connection->EnqueueMsg(msgs::Package("request", request), true);

    try
    {
      connection->Read(data);
    }
    catch(...)
    {
      gzerr << "An active gzserver is probably not present.\n";
      connection.reset();
      return;
    }

    packet.ParseFromString(data);
    pubs.ParseFromString(packet.serialized_data());

    // This list is used to filter topic output.
    std::list<std::string> listed;

    for (int i = 0; i < pubs.publisher_size(); i++)
    {
      const msgs::Publish &p = pubs.publisher(i);
      if (std::find(listed.begin(), listed.end(), p.topic()) == listed.end())
      {
        std::cout << p.topic() << std::endl;

        // Record the topics that have been listed to prevent duplicates.
        listed.push_back(p.topic());
      }
    }
  }

  connection.reset();
}

/////////////////////////////////////////////////
void echoCB(const std::string &_data)
{
  g_echoMsg->ParseFromString(_data);
  if (g_useShortDebugString)
    std::cout << g_echoMsg->ShortDebugString() << std::endl;
  else
    std::cout << g_echoMsg->DebugString() << std::endl;
}

/////////////////////////////////////////////////
void bwCB(const std::string &_data)
{
  boost::mutex::scoped_lock lock(mutex);

  bwBytes.push_back(_data.size());
  bwTime.push_back(common::Time::GetWallTime());
}

/////////////////////////////////////////////////
void hzCB(const std::string &/*_data*/)
{
  common::Time cur_time = common::Time::GetWallTime();

  if (hz_prev_time != common::Time(0, 0))
  {
    std::cout << "Hz: " << std::setw(6) << std::fixed << std::setprecision(2)
              << 1.0 / (cur_time - hz_prev_time).Double() << std::endl;
  }

  hz_prev_time = cur_time;
}

/////////////////////////////////////////////////
msgs::TopicInfo get_topic_info(const std::string &_topic)
{
  msgs::TopicInfo topic_info;
  std::string data;
  msgs::Request *request = msgs::CreateRequest("topic_info", _topic);
  msgs::Packet packet;

  transport::ConnectionPtr connection = connect_to_master();

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

  std::string topic = params[1];

  if (!transport::init())
    return;

  transport::NodePtr node(new transport::Node());
  node->Init();

  // Get the message type on the topic.
  std::string msgTypeName = gazebo::transport::getTopicMsgType(
      node->DecodeTopicName(topic));

  if (msgTypeName.empty())
  {
    gzerr << "Unable to get message type for topic[" << topic << "]\n";
    transport::fini();
    return;
  }

  g_echoMsg = msgs::MsgFactory::NewMsg(msgTypeName);

  if (!g_echoMsg)
  {
    gzerr << "Unable to create message of type[" << msgTypeName << "]\n";
    transport::fini();
    return;
  }

  transport::SubscriberPtr sub = node->Subscribe(topic, echoCB);

  // Run the transport loop: starts a new thread
  transport::run();

  while (true)
    common::Time::MSleep(10);

  transport::fini();
}

/////////////////////////////////////////////////
void bw()
{
  if (params[1].empty())
  {
    std::cerr << "Error: No topic specified.\n";
    return;
  }

  if (!transport::init())
    return;

  transport::NodePtr node(new transport::Node());
  node->Init();

  std::string topic = params[1];

  transport::SubscriberPtr sub = node->Subscribe(topic, bwCB);

  // Run the transport loop: starts a new thread
  transport::run();

  while (true)
  {
    common::Time::MSleep(100);
    {
      boost::mutex::scoped_lock lock(mutex);
      if (bwBytes.size() >= 10)
      {
        std::sort(bwBytes.begin(), bwBytes.end());

        float sumSize = 0;
        unsigned int count = bwBytes.size();
        common::Time dt = bwTime[count - 1] - bwTime[0];

        for (unsigned int i = 0; i < count; ++i)
        {
          sumSize += bwBytes[i];
        }

        float meanBytes = sumSize / count;
        float totalBps = sumSize / dt.Double();

        // Create the output streams
        std::ostringstream bandwidth, mean, min, max;
        bandwidth << std::fixed << std::setprecision(2);
        mean << std::fixed << std::setprecision(2);
        min << std::fixed << std::setprecision(2);
        max << std::fixed << std::setprecision(2);

        // Format the bandwidth output
        if (totalBps < 1000)
          bandwidth << totalBps << " B/s";
        else if (totalBps < 1000000)
          bandwidth << totalBps / 1024.0f << " KB/s";
        else
          bandwidth << totalBps/1.049e6 << " MB/s";

        // Format message size  output
        if (meanBytes < 1000)
        {
          mean << meanBytes << " B";
          min << bwBytes[0] << " B";
          max << bwBytes[count-1] << " B";
        }
        else if (meanBytes < 1000000)
        {
          mean << meanBytes / 1024.0f << " KB";
          min << bwBytes[0] / 1024.0f << " KB";
          max << bwBytes[count-1] / 1024.0f << " KB";
        }
        else
        {
          mean << meanBytes / 1.049e6 << " MB";
          min << bwBytes[0] / 1.049e6 << " MB";
          max << bwBytes[count-1] / 1.049e6 << " MB";
        }

        std::cout << "Total[" << bandwidth.str() << "] "
                  << "Mean[" << mean.str() << "] "
                  << "Min[" << min.str() << "] "
                  << "Max[" << max.str() << "] "
                  << "Messages[" << count << "]\n";

        bwBytes.clear();
        bwTime.clear();
      }
    }
  }

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

  if (!transport::init())
    return;

  transport::NodePtr node(new transport::Node());
  node->Init();

  std::string topic = params[1];

  transport::SubscriberPtr sub = node->Subscribe(topic, hzCB);

  // Run the transport loop: starts a new thread
  transport::run();

  while (true)
    common::Time::MSleep(10);

  transport::fini();
}

/////////////////////////////////////////////////
void view(int _argc, char **_argv)
{
  if (!gazebo::setupClient())
  {
    printf("load error\n");
    return;
  }

  QApplication *app = new QApplication(_argc, _argv);

  QFile file(":/style.qss");
  file.open(QFile::ReadOnly);
  QString styleSheet = QLatin1String(file.readAll());

  app->setStyleSheet(styleSheet);

  std::string topic;
  std::string msgType;

  if (params.size() == 1 || params[1].empty())
  {
    gui::TopicSelector *selector = new gui::TopicSelector();
    selector->exec();

    topic = selector->GetTopic();
    msgType = selector->GetMsgType();
    delete selector;
  }
  else
  {
    topic = params[1];
    msgType = transport::getTopicMsgType(topic);
  }

  if (!topic.empty() && !msgType.empty())
  {
    gui::ViewFactory::RegisterAll();
    gui::TopicView *view = gui::ViewFactory::NewView(msgType, topic);
    if (view)
      view->show();
    else
      gzerr << "Unable to create viewer for message type[" << msgType << "]\n";
  }

  app->exec();

  gazebo::shutdown();
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  if (!parse(argc, argv))
    return 0;

  gazebo::common::Console::SetQuiet(true);

  if (params[0] == "list")
    list();
  else if (params[0] == "info")
    print_topic_info(params[1]);
  else if (params[0] == "echo")
    echo();
  else if (params[0] == "hz")
    hz();
  else if (params[0] == "bw")
    bw();
  else if (params[0] == "view")
    view(argc, argv);
}
