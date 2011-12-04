#include "ServerFixture.hh"

using namespace gazebo;

class TransportTest : public ServerFixture 
{};


TEST_F(TransportTest, Load)
{
  for (unsigned int i=0; i < 10; i++)
  {
    Load("worlds/empty.world");
    Unload();
  }
}

void ReceiveSceneMsg(const boost::shared_ptr<msgs::Scene const> &/*_msg*/)
{
}

TEST_F(TransportTest, PubSub)
{
  Load("worlds/empty.world");

  transport::NodePtr node = transport::NodePtr(new transport::Node());
  node->Init();
  transport::PublisherPtr scenePub = node->Advertise<msgs::Scene>("~/scene");
  transport::SubscriberPtr sceneSub = node->Subscribe("~/scene",
      &ReceiveSceneMsg);

  msgs::Scene msg;
  msgs::Init(msg, "test");
  msg.set_name("default");

  scenePub->Publish(msg);

  std::vector<transport::PublisherPtr> pubs;
  std::vector<transport::SubscriberPtr> subs;

  for (unsigned int i=0; i < 100; i++)
  {
    pubs.push_back(node->Advertise<msgs::Scene>("~/scene") );
    subs.push_back(node->Subscribe("~/scene", &ReceiveSceneMsg) );
    scenePub->Publish(msg);
  }

  pubs.clear();
  subs.clear();
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
