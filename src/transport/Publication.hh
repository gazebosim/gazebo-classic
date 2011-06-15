#ifndef PUBLICATION_HH
#define PUBLICATION_HH

#include <boost/shared_ptr.hpp>
#include <list>

#include "CallbackHelper.hh"
#include "PublicationTransport.hh"

namespace gazebo
{
  namespace transport
  {
    class Publication
    {
      /// \brief Constructor
      public: Publication( const std::string &topic, 
                           const std::string &msgType );

      /// \brief Destructor
      public: virtual ~Publication();

      /// \brief Get the topic for this publication
      public: std::string GetTopic() const;

      /// \brief Get the type of message
      public: std::string GetMsgType() const;

      public: void AddSubscription(const CallbackHelperPtr &callback);

      public: void RemoveSubscription(const CallbackHelperPtr &callback);

      /// \brief Remove a subscription
      public: void RemoveSubscription(const std::string &host, unsigned int port);

      public: void RemoveTransport(const std::string &host, unsigned int port);

      /// \brief Publish data
      public: void Publish(const std::string &data);
      public: void LocalPublish(const std::string &data);

      public: void Publish(const google::protobuf::Message &msg,
                           const boost::function<void()> &cb = NULL);

      public: void AddTransport( const PublicationTransportPtr &publink );

      private: std::string topic;
      private: std::string msgType;
      private: std::list< CallbackHelperPtr > callbacks;

      private: std::list<PublicationTransportPtr> transports;

      private: google::protobuf::Message *prevMsg;
    };
    typedef boost::shared_ptr<Publication> PublicationPtr;
  }
}
#endif
