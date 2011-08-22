#ifndef MASTER_HH
#define MASTER_HH

#include <string>
#include <list>
#include <deque>
#include <boost/shared_ptr.hpp>

#include "msgs/msgs.h"
#include "transport/Connection.hh"

namespace gazebo
{

  class Master
  {
    public: Master();
    public: virtual ~Master();

    public: void Init(unsigned short port);
    public: void Run();
    public: void Stop();
    public: void Fini();

    public: void RunLoop();

    private: void ProcessMessage(const unsigned int _connectionIndex,
                                 const std::string &_data);

    private: void OnRead(const unsigned int connectionIndex,
                         const std::string &data);

    private: void OnAccept(const transport::ConnectionPtr &new_connection);

    private: msgs::Publish GetPublisher( const std::string &topic );

    private: transport::ConnectionPtr FindConnection(const std::string &host, unsigned short port);

    private: std::deque<transport::ConnectionPtr> connections;

    typedef std::list< std::pair<msgs::Publish, transport::ConnectionPtr> > PubList;
    typedef std::list< std::pair<msgs::Subscribe, transport::ConnectionPtr> > SubList;
    private: PubList publishers;
    private: SubList subscribers;

    private: std::list< std::string > worldNames;
    private: std::list< std::pair<unsigned int, std::string> > msgs;

    private: transport::ConnectionPtr connection;
    private: boost::thread *runThread;
    private: bool stop;

    private: boost::recursive_mutex *connectionMutex, *msgsMutex;
  };
}

#endif
