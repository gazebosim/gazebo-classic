#ifndef PUBLICATION_HH
#define PUBLICATION_HH

#include <boost/shared_ptr.hpp>
#include <list>

#include "transport/CallbackHelper.hh"
#include "transport/TransportTypes.hh"
#include "transport/PublicationTransport.hh"

namespace gazebo
{
  namespace transport
  {
    /// \addtogroup gazebo_transport
    /// \{

    /// \brief A publication for a topic. This facilitates transport of
    ///messages
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

      public: unsigned int GetTransportCount();
      public: unsigned int GetCallbackCount();
      public: unsigned int GetRemoteSubscriptionCount();

      /// \brief Return true if the topic has been advertised from this
      ///        process.
      public: bool GetLocallyAdvertised() const;

      /// \brief Set whether this topic has been advertised from this process
      public: void SetLocallyAdvertised(bool _value);

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

      private: bool locallyAdvertised;
    };
    /// \}
  }
}
#endif
