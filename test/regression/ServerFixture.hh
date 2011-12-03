#include <gtest/gtest.h>
#include <boost/thread.hpp>
#include "msgs/msgs.h"
#include "math/MathTypes.hh"
#include "src/Server.hh"

using namespace gazebo;

class ServerFixture : public testing::Test
{
  protected: ServerFixture()
             {
               this->receiveMutex = new boost::mutex();
               this->server = NULL;
             }

  protected: virtual void TearDown()
             {
               this->node->Fini();

               if (this->server)
                 this->server->Stop();

               if (this->serverThread)
                 this->serverThread->join();

               delete this->receiveMutex;
             }

  protected: virtual void Load(const std::string &_worldFilename)
             {
               if (this->server)
                 delete this->server;
               this->server = NULL;

               // Create, load, and run the server in its own thread
               this->serverThread = new boost::thread(
                  boost::bind(&ServerFixture::RunServer, this, _worldFilename));

               // Wait for the server to come up
               while (!this->server || !this->server->GetInitialized())
                 usleep(10000);

               this->node = transport::NodePtr(new transport::Node());
               ASSERT_NO_THROW(this->node->Init());
               this->poseSub = this->node->Subscribe("~/pose/info",
                   &ServerFixture::OnPose, this);
             }

  protected: void RunServer(const std::string &_worldFilename)
             {
               ASSERT_NO_THROW(this->server = new Server());
               ASSERT_NO_THROW(this->server->Load(_worldFilename));
               ASSERT_NO_THROW(this->server->Init());
               this->server->Run();
               ASSERT_NO_THROW(this->server->Fini());
               delete this->server;
             }

  protected: void OnPose(const boost::shared_ptr<msgs::Pose const> &_msg)
             {
               this->receiveMutex->lock();
               this->poses[_msg->name()] = msgs::Convert(*_msg);
               this->receiveMutex->unlock();
             }

  protected: math::Pose GetEntityPose(const std::string &_name)
             {
               std::map<std::string, math::Pose>::iterator iter;
               this->receiveMutex->lock();
               iter = this->poses.find(_name);
               EXPECT_TRUE(iter != this->poses.end());
               this->receiveMutex->unlock();
               return iter->second;
             }

  protected: bool HasEntity(const std::string &_name)
             {
               std::map<std::string, math::Pose>::iterator iter;
               this->receiveMutex->lock();
               iter = this->poses.find(_name);
               this->receiveMutex->unlock();
               return iter != this->poses.end();
             }

  protected: Server *server;
  protected: boost::thread *serverThread;
  protected: transport::NodePtr node;
  protected: transport::SubscriberPtr poseSub;
  protected: std::map<std::string, math::Pose> poses;
  protected: boost::mutex *receiveMutex;
};
