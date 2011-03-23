#ifndef MASTER_HH
#define MASTER_HH

#include <string>
#include <list>
#include <boost/shared_ptr.hpp>

#include "common/Messages.hh"
#include "transport/Connection.hh"

namespace gazebo
{

  class Master
  {
    public: Master();
    public: virtual ~Master();

    public: void Init(unsigned short port);
    public: void Run();
    public: void Quit();

    private: void OnRead(const std::string &data);

    private: void HandlePublish(const boost::shared_ptr<msgs::Publish const> &msg);

    private: void HandleSubscribe(const boost::shared_ptr<msgs::Subscribe const> &msg);

    private: void OnAccept(const transport::ConnectionPtr &new_connection);

    private: std::list<transport::ConnectionPtr> connections;
    private: std::list<msgs::Publish> publishers;
    private: std::list<msgs::Subscribe> subscribers;
    private: transport::ConnectionPtr connection;
    private: bool quit;
  };
}

#endif
