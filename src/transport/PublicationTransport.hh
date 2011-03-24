#ifndef PUBLICATIONLINK_HH
#define PUBLICATIONLINK_HH

#include <boost/shared_ptr.hpp>

#include "Connection.hh"
#include "CallbackHelper.hh"

namespace gazebo
{
  namespace transport
  {
    class PublicationTransport : public CallbackHelper
    {
      /// \brief Constructor
      public: PublicationTransport();

      /// \brief Destructor
      public: virtual ~PublicationTransport();

      /// \brief Initialize the publication link 
      public: void Init( const ConnectionPtr &conn );

      /// \brief Get the typename of the message that is handled
      public: virtual std::string GetMsgType() const;

      /// \brief Output a message to a connection
      public: virtual void HandleMessage(const std::string &newdata);

      private: ConnectionPtr connection;
    };
    typedef boost::shared_ptr<PublicationTransport> PublicationTransportPtr;
  }
}

#endif
