#include <gtest/gtest.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>

#include "src/Server.hh"
#include "transport/TransportTypes.hh"
#include "transport/Node.hh"


using namespace gazebo;

double g_percent = 0;

std::string g_worldFile;
Server *server = NULL;

void OnStats(const boost::shared_ptr<msgs::WorldStatistics const> &_msg)
{
  common::Time simTime  = msgs::Convert( _msg->sim_time() );
  common::Time realTime = msgs::Convert( _msg->real_time() );

  if (simTime.Double() > 0)
    g_percent  = (simTime / realTime).Double();
}

void RunServer()
{
  ASSERT_NO_THROW(server = new Server());
  ASSERT_NO_THROW(server->Load(g_worldFile));
  ASSERT_NO_THROW(server->Init());

  server->Run();
  ASSERT_NO_THROW(server->Fini());
  delete server;
}

void SpeedTest(double minSpeed)
{
  boost::thread thread(boost::bind(RunServer));
  while (!server || !server->GetInitialized())
    usleep(1000000);

  transport::NodePtr node = transport::NodePtr(new transport::Node());
  ASSERT_NO_THROW(node->Init());
  transport::SubscriberPtr sub = node->Subscribe("~/world_stats", OnStats);

  g_percent = -1;
  while (g_percent < 0)
    usleep(100000);

  EXPECT_GT(g_percent, minSpeed);

  ASSERT_NO_THROW(server->Stop());
  thread.join();

  usleep(1000000);
}

TEST(Speed, EmptyWorld)
{
  g_worldFile = "worlds/empty.world";
  SpeedTest(3800.0);
}

TEST(Speed, ShapesWorld)
{
  g_worldFile = "worlds/shapes.world";
  SpeedTest(120.0);
}

TEST(Speed, PR2World)
{
  g_worldFile = "worlds/pr2.world";
  SpeedTest(4.0);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
