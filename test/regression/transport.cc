#include <gtest/gtest.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>

#include "src/Server.hh"
#include "src/Master.hh"
#include "transport/TransportTypes.hh"
#include "transport/Connection.hh"
#include "transport/Transport.hh"
#include "transport/Node.hh"

#include "gazebo_config.h"

using namespace gazebo;
class ServerTest : public testing::Test
{
  protected: virtual void SetUp()
             {
               this->serverThread = NULL;
               this->server = new Server();
               EXPECT_TRUE(this->server != NULL);
             }

  protected: virtual void TearDown()
             {
               this->Unload();
               delete this->server;
               this->server = NULL;
             }

  protected: virtual void Load(const std::string &_worldFilename)
             {
               ASSERT_TRUE(this->server->Load(_worldFilename));
               this->server->Init();

               this->serverThread = new boost::thread(
                   boost::bind(&Server::Run, this->server));
             }

  protected: virtual void Unload()
             {
               this->server->Fini();
               if (this->serverThread)
               {
                 this->serverThread->join();
               }
               delete this->serverThread;
               this->serverThread = NULL;
             }

  protected: Server *server;
  protected: boost::thread *serverThread;
};

class MasterTest : public testing::Test
{
  protected: virtual void SetUp()
             {
               std::string host = "";
               unsigned short port = 0;

               transport::get_master_uri(host,port);

               this->master = new Master();
               this->master->Init(port);
               this->master->Run();
             }
  protected: virtual void TearDown()
             {
               this->master->Fini();
               delete this->master;
               this->master = NULL;
             }

  protected: transport::ConnectionPtr ConnectToMaster()
             {
               std::string data, namespacesData, publishersData;
               msgs::Packet packet;

               std::string host = "";
               unsigned short port = 0;

               transport::get_master_uri(host,port);

               // Connect to the master
               transport::ConnectionPtr connection( new transport::Connection() );
               connection->Connect(host, port);

               // Read the verification message
               connection->Read(data);
               connection->Read(namespacesData);
               connection->Read(publishersData);

               packet.ParseFromString(data);
               if (packet.type() == "init")
               {
                 msgs::String msg;
                 msg.ParseFromString( packet.serialized_data() );
                 if (msg.data() != std::string("gazebo ") + GAZEBO_VERSION)
                   std::cerr << "Conflicting gazebo versions\n";
               }

               return connection;
             }

  protected: Master *master;
};

TEST_F(MasterTest, MasterConstructor)
{
  EXPECT_TRUE(master != NULL);
}

TEST_F(MasterTest, MasterConnect)
{
  for (unsigned int i=0; i < 100; i++)
  {
    transport::ConnectionPtr conn = ConnectToMaster();
    EXPECT_TRUE(conn != NULL);
    EXPECT_TRUE(conn->IsOpen());
  }
}


TEST_F(ServerTest, ServerConstructor)
{
  for (unsigned int i=0; i < 10; i++)
  {
    Load("worlds/empty.world");
    Unload();
  }
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
