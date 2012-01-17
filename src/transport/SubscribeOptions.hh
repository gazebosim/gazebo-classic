#ifndef SUBSCRIBEOPTIONS_HH
#define SUBSCRIBEOPTIONS_HH

#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include "transport/CallbackHelper.hh"

namespace gazebo
{
  namespace transport
  {
    /// \addtogroup gazebo_transport
    /// \{
    /// \brief Options for a subscription
    class SubscribeOptions
    {
      public: SubscribeOptions()
              {}
      public: template<class M>
              void Init(const std::string &_topic,
                        NodePtr _node,
                        bool _latching)
              {
                google::protobuf::Message *msg = NULL;
                M msgtype;
                msg = dynamic_cast<google::protobuf::Message *>(&msgtype);
                if (!msg)
                  gzthrow("Subscribe requires a google protobuf type");

                this->node = _node;
                this->topic = _topic;
                this->msgType = msg->GetTypeName();
                this->latching = _latching;
              }

      public: NodePtr GetNode() const
              {
                return this->node;
              }

      public: std::string GetTopic() const
              {
                return this->topic;
              }

      public: std::string GetMsgType() const
              {
                return this->msgType;
              }

      public: bool GetLatching() const
              {
                return this->latching;
              }

      private: std::string topic;
      private: std::string msgType;
      private: NodePtr node;
      private: bool latching;
    };
    /// \}
  }
}

#endif

