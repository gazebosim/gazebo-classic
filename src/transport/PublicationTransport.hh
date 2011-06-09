#ifndef PUBLICATIONTRANSPORT_HH
#define PUBLICATIONTRANSPORT_HH

#include <boost/shared_ptr.hpp>
#include "transport/Connection.hh"
#include "common/Event.hh"

namespace gazebo
{
  namespace transport
  {
    class PublicationTransport
    {
      public: PublicationTransport(const std::string &topic, 
                                   const std::string &msgType);

      public: virtual ~PublicationTransport();

      public: void Init(const ConnectionPtr &conn);
      public: void AddCallback(const boost::function<void(const std::string &)> &cb);

      public: const ConnectionPtr GetConnection() const;

      private: void OnConnectionShutdown();

      private: void OnPublish(const std::string &data);

      private: std::string topic;
      private: std::string msgType;
      private: ConnectionPtr connection;
      private: boost::function<void (const std::string &)> callback;
      private: event::ConnectionPtr shutdownConnectionPtr;
      //private: PublicationPtr pub;
    };
    typedef boost::shared_ptr<PublicationTransport> PublicationTransportPtr;
  }
}

#endif
