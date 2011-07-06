#ifndef SUBSCRIBEOPTIONS_HH
#define SUBSCRIBEOPTIONS_HH

#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include "transport/CallbackHelper.hh"

namespace gazebo
{
  namespace transport
  {
    class SubscribeOptions
    {
      public: SubscribeOptions()
              {}

      public: template<class M>
              void Init(const std::string &_topic, 
                        const boost::function<void (const boost::shared_ptr<M const> &)> &_callback)
              {
                google::protobuf::Message *msg = NULL;
                M msgtype;
                msg = dynamic_cast<google::protobuf::Message *>(&msgtype);
                if (!msg)
                  gzthrow("Subscribe requires a google protobuf type");
 
                this->topic = _topic;
                this->msgType = msg->GetTypeName();
                  this->subscription = CallbackHelperPtr( 
                      new CallbackHelperT<M>(_callback) );
              }

      public: std::string GetTopic() const
              {
                return this->topic;
              }

      public: std::string GetMsgType() const
              {
                return this->msgType;
              }

      public: CallbackHelperPtr GetSubscription() const
              {
                return this->subscription;
              }

      private: std::string topic;
      private: std::string msgType;
      private: CallbackHelperPtr subscription;
    };
  }
}

#endif
