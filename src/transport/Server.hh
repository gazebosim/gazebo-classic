#ifndef SERVER_HH
#define SERVER_HH

#include <boost/thread.hpp>
#include <boost/asio.hpp>

#include "common/Messages.hh"

#include "transport/Connection.hh"
#include "transport/Publisher.hh"

namespace gazebo
{
  namespace transport
  {
    class Server
    {
      public: Server(unsigned short port);
      public: void OnAccept(const boost::system::error_code &e, ConnectionPtr conn);
      public: void OnWrite(const boost::system::error_code &e, ConnectionPtr conn);

      /*public: template<typename M>
              PublisherPtr Advertise(const std::string &topic_name)
              {
                common::MessageType type = M::GetType();

                PublisherPtr pub( new Publisher(topic_name, type) );
                this->publishers.insert( std::make_pair(topic_name, pub) );

                return pub;
              }*/

      public: void Write(const std::string &msg);

      private: boost::asio::ip::tcp::acceptor acceptor;

      private: std::map<std::string, PublisherPtr> publishers;
      private: std::vector<ConnectionPtr> connections;
      private: std::string hostname;
      private: unsigned short port;
    };
  }
}

#endif
