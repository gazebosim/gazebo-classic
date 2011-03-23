#ifndef MASTER_HH
#define MASTER_HH

#include <string>
#include <list>
#include <boost/shared_ptr.hpp>

#include "common/Messages.hh"

namespace gazebo
{
  namespace transport
  {
    class Server;
  }

  class Master
  {
    public: Master();
    public: virtual ~Master();

    public: void Init(unsigned short port);
    public: void Run();
    public: void Quit();


    private: void HandlePublish(const boost::shared_ptr<msgs::Publish const> &msg);

    private: void HandleSubscribe(const boost::shared_ptr<msgs::Subscribe const> &msg);

    private: std::list<msgs::Publish> publishers;
    private: std::list<msgs::Subscribe> subscribers;
    private: transport::Server *server;
    private: bool quit;
  };
}

#endif
