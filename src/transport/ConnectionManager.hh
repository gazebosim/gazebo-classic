#ifndef CONNECTION_MANAGER_HH
#define CONNECTION_MANAGER_HH

namespace gazebo
{
  namespace transport
  {
    class ConnectionManager : public SingletonT<ConnectionManager>
    {
      private: ConnectionManager();
      private: virtual ~ConnectionManager();

      public: void Run();

      private: ConnectionPtr connection;
    };
  }
}

#endif
