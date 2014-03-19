/*
 * Copyright 2012-2014 Open Source Robotics Foundation
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

#include <gazebo/gui/qt.h>
#include <gazebo/gui/TopicSelector.hh>
#include <gazebo/gui/viewers/TopicView.hh>
#include <gazebo/gui/viewers/ViewFactory.hh>
#include <gazebo/gazebo.hh>

#include "gz_topic.hh"

using namespace gazebo;

/////////////////////////////////////////////////
TopicCommand::TopicCommand()
  : Command("topic", "Lists information about topics on a Gazebo master")
{
  // Options that are visible to the user through help.
  this->visibleOptions.add_options()
    ("world-name,w", po::value<std::string>(), "World name.")
    ("list,l", "List all topics.")
    ("info,i", po::value<std::string>(), "Get information about a topic.")
    ("echo,e", po::value<std::string>(), "Output topic data to screen.")
    ("view,v", po::value<std::string>()->implicit_value(""),
     "View topic data using a QT widget.")
    ("hz,z", po::value<std::string>(), "Get publish frequency.")
    ("bw,b", po::value<std::string>(), "Get topic bandwidth.")
    ("unformatted,u", "Output data from echo without formatting.")
    ("duration,d", po::value<double>(), "Duration (seconds) to run. "
     "Applicable with echo, hz, and bw");
}

/////////////////////////////////////////////////
void TopicCommand::HelpDetailed()
{
  std::cerr <<
    "\tPrint topic information to standard out. If a name for the world, \n"
    "\toption -w, is not specified, the first world found on \n"
    "\tthe Gazebo master will be used.\n"
    << std::endl;
}

/////////////////////////////////////////////////
bool TopicCommand::RunImpl()
{
  std::string worldName;

  if (this->vm.count("world-name"))
    worldName = this->vm["world-name"].as<std::string>();

  this->node.reset(new transport::Node());
  this->node->Init(worldName);

  if (this->vm.count("list"))
    this->List();
  else if (this->vm.count("info"))
    this->Info(this->vm["info"].as<std::string>());
  else if (this->vm.count("echo"))
    this->Echo(this->vm["echo"].as<std::string>());
  else if (this->vm.count("hz"))
    this->Hz(this->vm["hz"].as<std::string>());
  else if (this->vm.count("bw"))
    this->Bw(this->vm["bw"].as<std::string>());
  else if (this->vm.count("view"))
    this->View(this->vm["view"].as<std::string>());
  else
    this->Help();

  return true;
}

/////////////////////////////////////////////////
void TopicCommand::List()
{
  std::string data;
  msgs::Packet packet;
  msgs::Request request;
  msgs::GzString_V topics;

  transport::ConnectionPtr connection = transport::connectToMaster();

  if (connection)
  {
    request.set_id(0);
    request.set_request("get_topics");
    connection->EnqueueMsg(msgs::Package("request", request), true);
    connection->Read(data);

    packet.ParseFromString(data);
    topics.ParseFromString(packet.serialized_data());

    for (int i = 0; i < topics.data_size(); ++i)
    {
      std::cout << topics.data(i) << std::endl;
    }
  }

  connection.reset();
}

/////////////////////////////////////////////////
msgs::TopicInfo TopicCommand::GetInfo(const std::string &_topic)
{
  msgs::TopicInfo topicInfo;
  std::string data;
  msgs::Request *request = msgs::CreateRequest("topic_info", _topic);
  msgs::Packet packet;

  transport::ConnectionPtr connection = transport::connectToMaster();

  connection->EnqueueMsg(msgs::Package("request", *request), true);

  int i = 0;
  do
  {
    connection->Read(data);
    packet.ParseFromString(data);
  } while (packet.type() != "topic_info_response" && ++i < 10);

  if (i <10)
    topicInfo.ParseFromString(packet.serialized_data());
  else
    std::cerr << "Unable to get topic info.\n";

  delete request;
  return topicInfo;
}

/////////////////////////////////////////////////
void TopicCommand::Info(const std::string &_topic)
{
  msgs::TopicInfo info = this->GetInfo(_topic);
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
void TopicCommand::EchoCB(const std::string &_data)
{
  this->echoMsg->ParseFromString(_data);
  if (this->vm.count("unformatted") > 0)
    std::cout << this->echoMsg->ShortDebugString() << "\n";
  else
    std::cout << this->echoMsg->DebugString() << "\n";
}

/////////////////////////////////////////////////
void TopicCommand::Echo(const std::string &_topic)
{
  // Get the message type on the topic.
  std::string msgTypeName = gazebo::transport::getTopicMsgType(
      this->node->DecodeTopicName(_topic));

  if (msgTypeName.empty())
  {
    gzerr << "Unable to get message type for topic[" << _topic << "]\n";
    return;
  }

  this->echoMsg = msgs::MsgFactory::NewMsg(msgTypeName);

  if (!this->echoMsg)
  {
    gzerr << "Unable to create message of type[" << msgTypeName << "]\n";
    transport::fini();
    return;
  }

  transport::SubscriberPtr sub = this->node->Subscribe(_topic,
      &TopicCommand::EchoCB, this);

  boost::mutex::scoped_lock lock(this->sigMutex);
  if (this->vm.count("duration"))
    this->sigCondition.timed_wait(lock,
        boost::posix_time::seconds(this->vm["duration"].as<double>()));
  else
    this->sigCondition.wait(lock);
}

/////////////////////////////////////////////////
void TopicCommand::HzCB(const std::string &/*_data*/)
{
  common::Time curTime = common::Time::GetWallTime();

  if (this->prevMsgTime != common::Time(0, 0))
    printf("Hz: %6.2f\n", 1.0 / (curTime - this->prevMsgTime).Double());

  this->prevMsgTime = curTime;
}

/////////////////////////////////////////////////
void TopicCommand::Hz(const std::string &_topic)
{
  transport::SubscriberPtr sub = this->node->Subscribe(_topic,
      &TopicCommand::HzCB, this);

  boost::mutex::scoped_lock lock(this->sigMutex);
  if (this->vm.count("duration"))
    this->sigCondition.timed_wait(lock,
        boost::posix_time::seconds(this->vm["duration"].as<double>()));
  else
    this->sigCondition.wait(lock);
}

/////////////////////////////////////////////////
void TopicCommand::BwCB(const std::string &_data)
{
  common::Time curTime = common::Time::GetWallTime();

  this->bwBytes.push_back(_data.size());
  this->bwTime.push_back(common::Time::GetWallTime());

  // One second time window
  if (curTime - this->prevMsgTime > common::Time(1, 0))
  {
    // Make sure we have received at least 10 bytes of data.
    if (this->bwBytes.size() >= 10)
    {
      std::sort(this->bwBytes.begin(), this->bwBytes.end());

      float sumSize = 0;
      unsigned int count = this->bwBytes.size();
      common::Time dt = this->bwTime[count - 1] - this->bwTime[0];

      for (unsigned int i = 0; i < count; ++i)
      {
        sumSize += this->bwBytes[i];
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
        min << this->bwBytes[0] << " B";
        max << this->bwBytes[count-1] << " B";
      }
      else if (meanBytes < 1000000)
      {
        mean << meanBytes / 1024.0f << " KB";
        min << this->bwBytes[0] / 1024.0f << " KB";
        max << this->bwBytes[count-1] / 1024.0f << " KB";
      }
      else
      {
        mean << meanBytes / 1.049e6 << " MB";
        min << this->bwBytes[0] / 1.049e6 << " MB";
        max << this->bwBytes[count-1] / 1.049e6 << " MB";
      }

      std::cout << "Total[" << bandwidth.str() << "] "
        << "Mean[" << mean.str() << "] "
        << "Min[" << min.str() << "] "
        << "Max[" << max.str() << "] "
        << "Messages[" << count << "]\n";

      this->bwBytes.clear();
      this->bwTime.clear();
    }

    this->prevMsgTime = curTime;
  }
}

/////////////////////////////////////////////////
void TopicCommand::Bw(const std::string &_topic)
{
  this->prevMsgTime = common::Time::GetWallTime();
  transport::SubscriberPtr sub = node->Subscribe(_topic,
      &TopicCommand::BwCB, this);

  boost::mutex::scoped_lock lock(this->sigMutex);
  if (this->vm.count("duration"))
    this->sigCondition.timed_wait(lock,
        boost::posix_time::seconds(this->vm["duration"].as<double>()));
  else
    this->sigCondition.wait(lock);
}

/////////////////////////////////////////////////
void TopicCommand::View(const std::string &_topic)
{
  if (!gazebo::setupClient())
  {
    printf("load error\n");
    return;
  }

  QApplication *app = new QApplication(this->argc, this->argv);

  QFile file(":/style.qss");
  file.open(QFile::ReadOnly);
  QString styleSheet = QLatin1String(file.readAll());

  app->setStyleSheet(styleSheet);

  std::string topic;
  std::string msgType;

  if (_topic.empty())
  {
    gui::TopicSelector *selector = new gui::TopicSelector();
    selector->exec();

    topic = selector->GetTopic();
    msgType = selector->GetMsgType();
    delete selector;
  }
  else
  {
    topic = _topic;
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

  delete app;
  app = NULL;

  gazebo::shutdown();
}
