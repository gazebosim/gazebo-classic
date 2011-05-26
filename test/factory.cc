#include <gtest/gtest.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>

#include "src/Combined.hh"
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


std::string model_name;

boost::mutex mutex;
boost::condition_variable cond;

void OnNewEntity(const boost::shared_ptr<msgs::Entity const> &msg)
{
  model_name = msg->name();
  cond.notify_one();
}

TEST(Factory, BoxCreate)
{
  Combined *combined = NULL;

  boost::unique_lock<boost::mutex> lock(mutex);
  boost::posix_time::time_duration td = boost::posix_time::seconds(1);

  ASSERT_NO_THROW(combined = new Combined());
  ASSERT_NO_THROW(combined->Load(""));
  ASSERT_NO_THROW(combined->Init());

  combined->Start();

  transport::NodePtr node = transport::NodePtr( new transport::Node() );
  ASSERT_NO_THROW(node->Init("default"));

  transport::PublisherPtr pub = node->Advertise<msgs::Factory>("~/factory");
  transport::SubscriberPtr sub = node->Subscribe("~/new_entity", OnNewEntity);

  msgs::Factory msg;
  ASSERT_NO_THROW(common::Message::Init(msg, "new_box"));
  msg.set_xml( box_xml );

  // Publish the message
  ASSERT_NO_THROW(pub->Publish(msg));

  // Wait for the new entity message
  cond.timed_wait(lock, td);
 
  // Check the right model was created 
  ASSERT_STREQ(model_name.c_str(), "factory_box_test");
  
  ASSERT_NO_THROW(combined->Stop());

  delete combined;
}

TEST(Factory, SphereCreate)
{
  Combined *combined = NULL;

  boost::unique_lock<boost::mutex> lock(mutex);
  boost::posix_time::time_duration td = boost::posix_time::seconds(1);

  ASSERT_NO_THROW(combined = new Combined());
  ASSERT_NO_THROW(combined->Load(""));
  ASSERT_NO_THROW(combined->Init());

  combined->Start();

  transport::NodePtr node = transport::NodePtr( new transport::Node() );
  ASSERT_NO_THROW(node->Init("default"));

  transport::PublisherPtr pub = node->Advertise<msgs::Factory>("~/factory");
  transport::SubscriberPtr sub = node->Subscribe("~/new_entity", OnNewEntity);

  msgs::Factory msg;
  ASSERT_NO_THROW(common::Message::Init(msg, "new_box"));
  msg.set_xml( sphere_xml );

  // Publish the message
  ASSERT_NO_THROW(pub->Publish(msg));

  // Wait for the new entity message
  cond.timed_wait(lock, td);
 
  // Check the right model was created 
  ASSERT_STREQ(model_name.c_str(), "factory_sphere_test");
  
  ASSERT_NO_THROW(combined->Stop());

  delete combined;
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
