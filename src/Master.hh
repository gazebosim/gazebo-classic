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
    /// \brief Constructor
    public: Master();

    /// \brief Destructor
    public: virtual ~Master();

    /// \brief Initialize
    /// \param _port The master's port
    public: void Init(unsigned short _port);

    /// \brief Run the master in a new thread
    public: void Run();

    /// \brief Stop the master
    public: void Stop();

    /// \brief Finalize the master
    public: void Fini();

    /// \brief The run loop
    private: void RunLoop();

    /// \brief Process a message
    /// \param _connectionIndex Index of the connection which generated the
    ///                         message
    /// \param _data The message data
    private: void ProcessMessage(const unsigned int _connectionIndex,
                                 const std::string &_data);

    /// \brief Connection read callback
    /// \param _connectionIndex Index of the connection which generated the
    ///                         message
    /// \param _data The message data
    private: void OnRead(const unsigned int _connectionIndex,
                         const std::string &_data);

    /// \brief Accept a new connection
    /// \param _newConnection The new connection
    private: void OnAccept(const transport::ConnectionPtr &_newConnection);

    /// \brief Get a publisher for the given topic
    /// \param _topic Name of the topic
    /// \return A publish message
    private: msgs::Publish GetPublisher( const std::string &_topic );

    /// \brief Find a connection given a host and port
    /// \param _host Host name
    /// \param _port Port number
    /// \return The found connection, or NULL
    private: transport::ConnectionPtr FindConnection(const std::string &_host, 
                                                     unsigned short _port);


    typedef std::list< std::pair<msgs::Publish, transport::ConnectionPtr> > PubList;
    typedef std::list< std::pair<msgs::Subscribe, transport::ConnectionPtr> > SubList;
    private: PubList publishers;
    private: SubList subscribers;

    private: std::deque<transport::ConnectionPtr> connections;
    private: std::list< std::string > worldNames;
    private: std::list< std::pair<unsigned int, std::string> > msgs;

    private: transport::ConnectionPtr connection;
    private: boost::thread *runThread;
    private: bool stop;

    private: boost::recursive_mutex *connectionMutex, *msgsMutex;
  };
}

#endif
