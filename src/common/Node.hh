#ifndef NODE_HH
#define NODE_HH

#include <string>
#include <boost/shared_ptr.hpp>

#include "transport/Publisher.hh"
#include "transport/Connection.hh"
#include "transport/TopicManager.hh"

namespace gazebo
{
  namespace common
  {
    class Node
    {
      public: Node();
      public: virtual ~Node();

      public: void Init(const std::string &master_host, unsigned short master_port);

      public: template<typename M, typename T>
              transport::SubscriberPtr Subscribe(const std::string topic, 
                  void(T::*fp)(const boost::shared_ptr<M const> &), T *obj)
              {
                transport::SubscriberPtr sub;
                sub = transport::TopicManager::Instance()->Subscribe(topic, fp, obj);

                msgs::Subscribe msg;
                msg.set_topic( sub->GetTopic() );
                msg.set_msg_type( sub->GetMsgType() );
                msg.set_host( this->serverConn->GetLocalAddress() );
                msg.set_port( this->serverConn->GetLocalPort() );

                std::cout << "Node::Subscribe\n";
                this->masterConn->Write(Message::Package("subscribe", msg));

                return sub;
              }
 
      public: template<typename M>
              transport::PublisherPtr Advertise(const std::string topic)
              {
                std::cout << "Node::Advertise\n";

                transport::PublisherPtr pub;
                pub = transport::TopicManager::Instance()->Advertise<M>(topic);

                msgs::Publish msg;
                msg.set_topic( pub->GetTopic() );
                msg.set_msg_type( pub->GetMsgType() );
                msg.set_host( this->serverConn->GetLocalAddress() );
                msg.set_port( this->serverConn->GetLocalPort() );

                this->masterConn->Write(Message::Package("advertise", msg));

                return pub;
              }

      private: void OnMasterRead( const std::string &data );

      private: void OnAccept(const transport::ConnectionPtr &new_connection);

      private: void OnReadHeader(const transport::ConnectionPtr &new_connection,
                                 const std::string &data );

      private: transport::ConnectionPtr masterConn;
      private: transport::ConnectionPtr serverConn;

      private: std::list<transport::ConnectionPtr> subConnections;
    };

    typedef boost::shared_ptr<Node> NodePtr;
  }
}

#endif
