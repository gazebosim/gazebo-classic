#ifndef PUBLISHER_HH
#define PUBLISHER_HH

#include <google/protobuf/message.h>
#include <boost/shared_ptr.hpp>

namespace gazebo
{
  class Publisher
  {
    public: Publisher() {}
    public: Publisher(const std::string &topic, const std::string &msg_type);
    public: virtual ~Publisher();

    public: void Publish( const google::protobuf::Message &message );

    private: std::string topic;
    private: std::string msg_type;
  };
  typedef boost::shared_ptr<Publisher> PublisherPtr;
}

#endif
