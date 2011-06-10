#include <gtest/gtest.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>

#include "src/Server.hh"
#include "transport/TransportTypes.hh"
#include "transport/Node.hh"


using namespace gazebo;

const std::string box_xml = 
"<?xml version='1.0'?>\
<model name='factory_box_test'>\
  <origin xyz='2.89366 2.99339 1.57'/>\
  <link name='body'>\
    <collision name='geom'>\
      <geometry>\
        <box size='1.78731 1.98679 3.14'/>\
      </geometry>\
      <mass>0.5</mass>\
    </collision>\
    <visual>\
      <geometry>\
        <box size='1.78731 1.98679 3.14'/>\
      </geometry>\
      <material name='Gazebo/Grey'/>\
      <cast_shadows>true</cast_shadows>\
      <shader>pixel</shader>\
    </visual>\
  </link>\
</model>";

const std::string sphere_xml = 
"<?xml version='1.0'?>\
<model name='factory_sphere_test'>\
  <origin xyz='2.89366 2.99339 1.57'/>\
  <link name='body'>\
    <collision name='geom'>\
      <geometry>\
        <box size='1.78731 1.98679 3.14'/>\
      </geometry>\
      <mass>0.5</mass>\
    </collision>\
    <visual>\
      <geometry>\
        <box size='1.78731 1.98679 3.14'/>\
      </geometry>\
      <material name='Gazebo/Grey'/>\
      <cast_shadows>true</cast_shadows>\
      <shader>pixel</shader>\
    </visual>\
  </link>\
</model>";

const std::string cylinder_xml = 
"<?xml version='1.0'?>\
<model name='factory_cylinder_test'>\
  <origin xyz='2.89366 2.99339 1.57'/>\
  <link name='body'>\
    <collision name='geom'>\
      <geometry>\
        <box size='1.78731 1.98679 3.14'/>\
      </geometry>\
      <mass>0.5</mass>\
    </collision>\
    <visual>\
      <geometry>\
        <box size='1.78731 1.98679 3.14'/>\
      </geometry>\
      <material name='Gazebo/Grey'/>\
      <cast_shadows>true</cast_shadows>\
      <shader>pixel</shader>\
    </visual>\
  </link>\
</model>";



std::string model_name;

boost::mutex mutex;
boost::condition_variable cond;
bool triggered;

void OnNewEntity(const boost::shared_ptr<msgs::Entity const> &msg)
{
  model_name = msg->name();
  triggered = true;
  cond.notify_one();
}

TEST(Factory, BoxCreate)
{
  Server *server = NULL;

  boost::unique_lock<boost::mutex> lock(mutex);
  boost::posix_time::time_duration td = boost::posix_time::seconds(1);

  ASSERT_NO_THROW(server = new Server());
  ASSERT_NO_THROW(server->Load(""));
  ASSERT_NO_THROW(server->Init());

  //TODO: this is broken. Run has change to a blocking call.
  server->Run();

  transport::NodePtr node = transport::NodePtr( new transport::Node() );
  ASSERT_NO_THROW(node->Init("default"));

  transport::PublisherPtr pub = node->Advertise<msgs::Factory>("~/factory");
  transport::SubscriberPtr sub = node->Subscribe("~/new_entity", OnNewEntity);

  //------------------ Test Box Creation --------------------------------
  msgs::Factory msg;
  ASSERT_NO_THROW(common::Message::Init(msg, "new_box"));
  msg.set_xml( box_xml );

  triggered = false;
  // Publish the message
  ASSERT_NO_THROW(pub->Publish(msg));

  // Wait for the new entity message
  if (!triggered)
    cond.timed_wait(lock, td);
 
  // Check the right model was created 
  ASSERT_STREQ(model_name.c_str(), "factory_box_test");


  //------------------ Test Sphere Creation --------------------------------
  ASSERT_NO_THROW(common::Message::Init(msg, "new_sphere"));
  msg.set_xml( sphere_xml );

  triggered = false;

  // Publish the message
  ASSERT_NO_THROW(pub->Publish(msg));

  // Wait for the new entity message
  if (!triggered)
    cond.timed_wait(lock, td);
 
  // Check the right model was created 
  ASSERT_STREQ(model_name.c_str(), "factory_sphere_test");


  //------------------ Test Cylinder Creation --------------------------------
  ASSERT_NO_THROW(common::Message::Init(msg, "new_cylinder"));
  msg.set_xml( cylinder_xml );

  triggered = false;
  // Publish the message
  ASSERT_NO_THROW(pub->Publish(msg));

  // Wait for the new entity message
  if (!triggered)
    cond.timed_wait(lock, td);
 
  // Check the right model was created 
  ASSERT_STREQ(model_name.c_str(), "factory_cylinder_test");

  ASSERT_NO_THROW(server->Stop());
  ASSERT_NO_THROW(server->Fini());

  delete server;
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
