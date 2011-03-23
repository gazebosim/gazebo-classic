#ifndef SERVER_HH
#define SERVER_HH

#include <map>

#include <boost/function.hpp>
#include <boost/asio.hpp>
#include <google/protobuf/message.h>

#include "common/Messages.hh"

#include "transport/CallbackHelper.hh"
#include "transport/Connection.hh"
#include "transport/Publisher.hh"

namespace gazebo
{
  namespace transport
  {
    class Server
    {
      public: Server(unsigned short port);

      public: void ProcessIncoming();

      public: template<class M, class T>
              void Subscribe( const std::string &topic, 
                  void(T::*fp)(const boost::shared_ptr<M const>&), T *obj)
              {
                CallbackHelperT<M> *helper;
                helper = new CallbackHelperT<M>(boost::bind(fp, obj, _1));
                this->subscriptions[topic].push_back(helper);
              }

      public: void Write(const google::protobuf::Message &msg);

      public: int GetConnectionCount() const;

      private: void OnAccept(const boost::system::error_code &e, ConnectionPtr conn);

      private: boost::asio::ip::tcp::acceptor acceptor;
      private: std::vector<ConnectionPtr> connections;
      private: std::map<std::string, std::vector<CallbackHelper*> > subscriptions;
      private: std::string hostname;
      private: unsigned short port;
    };
  }
}

#endif
