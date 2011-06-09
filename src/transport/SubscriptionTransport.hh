#ifndef SUBSCRIPTIONTRANSPORT_HH
#define SUBSCRIPTIONTRANSPORT_HH

#include <boost/shared_ptr.hpp>

#include "Connection.hh"
#include "CallbackHelper.hh"

namespace gazebo
{
  namespace transport
  {
    class SubscriptionTransport : public CallbackHelper
    {
      /// \brief Constructor
      public: SubscriptionTransport();

      /// \brief Destructor
      public: virtual ~SubscriptionTransport();

      /// \brief Initialize the publication link 
      public: void Init( const ConnectionPtr &conn );

      /// \brief Get the typename of the message that is handled
      public: virtual std::string GetMsgType() const;

      public: virtual bool HandleMessage(const google::protobuf::Message *msg);

      /// \brief Output a message to a connection
      public: virtual bool HandleData(const std::string &newdata);

      /// \brief Get the connection
      public: const ConnectionPtr &GetConnection() const;

      private: ConnectionPtr connection;
    };
    typedef boost::shared_ptr<SubscriptionTransport> SubscriptionTransportPtr;
  }
}

#endif
