#include <gtest/gtest.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>

#include "src/Server.hh"
#include "transport/TransportTypes.hh"
#include "transport/Node.hh"


using namespace gazebo;

double g_percent = 0;

void OnStats(const boost::shared_ptr<msgs::WorldStatistics const> &_msg)
{
  common::Time simTime  = msgs::Convert( _msg->sim_time() );
  common::Time realTime = msgs::Convert( _msg->real_time() );

  if (simTime.Double() > 0)
    g_percent  = (simTime / realTime).Double();
}


TEST(Shapes, Regression)
{
  Server *server = NULL;

  ASSERT_NO_THROW(server = new Server());
  ASSERT_NO_THROW(server->Load("worlds/shapes.world"));
  ASSERT_NO_THROW(server->Init());

  boost::thread thread(boost::bind(&Server::Run, server));

  transport::NodePtr node = transport::NodePtr( new transport::Node() );
  ASSERT_NO_THROW(node->Init("default"));
  transport::SubscriberPtr sub = node->Subscribe("~/world_stats", OnStats);

  g_percent = -1;
  while (g_percent < 0)
    usleep(100000);

  EXPECT_GT(g_percent, 2.0);

  ASSERT_NO_THROW(server->Stop());
  thread.join();
  ASSERT_NO_THROW(server->Fini());

  delete server;

  printf("Sleeping\n");
  usleep(5000000);
  printf("DONE Sleeping\n");
}

TEST(PR2, Regression)
{
  Server *server = NULL;

  ASSERT_NO_THROW(server = new Server());
  ASSERT_NO_THROW(server->Load("worlds/pr2.world"));
  ASSERT_NO_THROW(server->Init());

  boost::thread thread(boost::bind(&Server::Run, server));

  transport::NodePtr node = transport::NodePtr( new transport::Node() );
  ASSERT_NO_THROW(node->Init("default"));
  transport::SubscriberPtr sub = node->Subscribe("~/world_stats", OnStats);

  g_percent = -1;
  while (g_percent < 0)
  {
    printf("Waiting\n");
    usleep(100000);
  }

  EXPECT_GT(g_percent, 2.0);

  ASSERT_NO_THROW(server->Stop());
  thread.join();
  ASSERT_NO_THROW(server->Fini());

  delete server;
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
