#include "GazeboError.hh"
#include "Simulator.hh"
#include "Publisher.hh"

using namespace gazebo;

Publisher::Publisher(const std::string &topic, const std::string &msg_type)
  : topic(topic), msg_type(msg_type)
{
  std::cout << "New Publisher. Topic[" << this->topic << "] MsgType[" << this->msg_type << "]\n";
}

Publisher::~Publisher()
{
  std::cout << "Publisher destructor[" << this->topic << "]\n";
  if (!this->topic.empty())
    Simulator::Instance()->Unadvertise(this->topic);
}

void Publisher::Publish(const google::protobuf::Message &message )
{
  if (message.GetTypeName() != this->msg_type)
    gzthrow("Invalid message type\n");

  std::cout << "Publish on topic[" << this->topic << "] Type[" << this->msg_type << "]\n";
  Simulator::Instance()->SendMessage(this->topic, this->msg_type, message);
}
