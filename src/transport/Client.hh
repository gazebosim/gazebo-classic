#ifndef CLIENT_HH
#define CLIENT_HH

#include <boost/asio.hpp>
#include <boost/shared_ptr.hpp>

#include "common/Messages.hh"
#include "transport/IOManager.hh"
#include "transport/CallbackHelper.hh"
#include "transport/Connection.hh"

namespace gazebo
{
  namespace transport
  {
    class Client
    {
      public: Client( const std::string &host, unsigned short port);

      public: const ConnectionPtr &GetConnection() const;

      public: void Write(const google::protobuf::Message &msg);

      private: void OnRead(const std::string &data);

      private: void OnReadInit(const std::string &data);

      private: ConnectionPtr connection;
      private: CallbackHelperPtr callback;

    };
    typedef boost::shared_ptr<Client> ClientPtr;
  }
}

#endif
