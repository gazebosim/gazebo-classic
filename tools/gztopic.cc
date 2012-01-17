#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include <google/protobuf/message.h>

#include "transport/Transport.hh"
#include "transport/TransportTypes.hh"
#include "transport/Node.hh"

#include "gazebo_config.h"

using namespace gazebo;

//transport::ConnectionPtr connection(new transport::Connection());
std::vector<std::string> params;

void help()
{
  std::cout << "Help message\n";
}

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
    msgs::String msg;
    msg.ParseFromString(packet.serialized_data());
    if (msg.data() != std::string("gazebo ") + GAZEBO_VERSION)
      std::cerr << "Conflicting gazebo versions\n";
  }

  return connection;
}

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

void echo_cb(ConstStringPtr &_data)
{
  std::cout << _data->data() << "\n";
}

msgs::TopicInfo get_topic_info(const std::string &_topic)
{
  msgs::TopicInfo topic_info;
  std::string data, type;
  msgs::Request request;
  msgs::Packet packet;

  transport::ConnectionPtr connection = connect_to_master("localhost", 11345);

  request.set_request("topic_info");
  request.set_data(_topic);
  connection->EnqueueMsg(msgs::Package("request", request), true);
  connection->Read(data);

  packet.ParseFromString(data);
  if (packet.type() == "topic_info_response")
  {
    topic_info.ParseFromString(packet.serialized_data());
  }
  else
    std::cerr << "Invalid response\n";

  return topic_info;
}

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

void echo()
{
  transport::init();

  transport::NodePtr node(new transport::Node());
  node->Init("default");

  std::string topic = params[1];
  topic +=  "/__dbg";

  transport::SubscriberPtr sub = node->Subscribe(topic, echo_cb);

  // Run the transport loop: starts a new thread
  transport::run();

  while(true)
    usleep(10000);

  transport::fini();
}

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
}
