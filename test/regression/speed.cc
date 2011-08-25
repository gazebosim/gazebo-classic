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

void SpeedTest(const std::string &worldFile, double minSpeed)
{
  Server *server = NULL;

  ASSERT_NO_THROW(server = new Server());
  ASSERT_NO_THROW(server->Load(worldFile));
  ASSERT_NO_THROW(server->Init());

  boost::thread thread(boost::bind(&Server::Run, server));

  transport::NodePtr node = transport::NodePtr( new transport::Node() );
  ASSERT_NO_THROW(node->Init());
  transport::SubscriberPtr sub = node->Subscribe("~/world_stats", OnStats);

  g_percent = -1;
  while (g_percent < 0)
    usleep(100000);

  EXPECT_GT(g_percent, minSpeed);

  ASSERT_NO_THROW(server->Stop());
  thread.join();
  ASSERT_NO_THROW(server->Fini());

  delete server;

  //TODO: this sleep should get removed...there is a timing issue with
  // cleanup. Some of the singletons don't shutdown in time.
  usleep(100000);
}

TEST(Speed, EmptyWorld)
{
  SpeedTest("worlds/empty.world", 1600.0);
}

TEST(Speed, ShapesWorld)
{
  SpeedTest("worlds/shapes.world", 1100.0);
}

TEST(Speed, PR2World)
{
  SpeedTest("worlds/pr2.world", 1.2);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
