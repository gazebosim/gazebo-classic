#ifndef CLIENT_HH
#define CLIENT_HH

#include <boost/asio.hpp>

#include "common/Message.hh"
#include "CallbackHelper.hh"
#include "transport/Connection.hh"

namespace gazebo
{
  namespace transport
  {
    class Client
    {
      public: Client( const std::string &host, const std::string &service);

      public: template<class M, class T>
              void Subscribe( const std::string &topic, void(T::*fp)(const M&), T *obj )
              {
                this->connection.reset( new Connection(IOManager::Instance()->GetIO()) );
                this->connection->Connect(this->host, this->service);

                //this->callback.reset( 
                    //new CallbackHelperT<M>(boost::bind(fp, obj, _1)));
                /*this->connection.async_read( 
                    boost::bind(&Client::OnRead, this, _1) );
                    */
              }

      private: void OnConnect(const boost::system::error_code &error,
                   boost::asio::ip::tcp::resolver::iterator endpoint_iter);

      private: void OnRead(const std::vector<char> &data);
      private: void OnReadInit(const std::vector<char> &data);

      private: ConnectionPtr connection;
      private: CallbackHelperPtr callback;
      private: std::map<std::string, MessageType > topics;
    };
  }
}

#endif
