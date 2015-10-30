/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <map>
#include <string>
#include <vector>

#include <ignition/math/Rand.hh>

#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/test/ServerFixture.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/test/helper_physics_generator.hh>

#define PHYSICS_TOL 1e-2
using namespace gazebo;

class PhysicsTest : public ServerFixture,
                    public testing::WithParamInterface<const char*>
{
  /// \brief test elastic modulus
  public: void ElasticModulusContact(const std::string &_physicsEngine);

  /// \brief Callback for contact subscribers in depth test.
  /// \param[in] _msg Contact message
  private: void Callback(const ConstContactsPtr &_msg);

  /// \brief Message to be filled with the latest contacts message.
  private: msgs::Contacts contactsMsg;

  /// \brief Mutex to protect reads and writes to contactsMsg.
  public: mutable boost::mutex mutex;
};

/////////////////////////////////////////////////
void PhysicsTest::Callback(const ConstContactsPtr &_msg)
{
  boost::mutex::scoped_lock lock(this->mutex);
  this->contactsMsg = *_msg;
}

////////////////////////////////////////////////////////////////////////
void PhysicsTest::ElasticModulusContact(const std::string &_physicsEngine)
{
  // check conservation of mementum for linear elastic collision
  Load("worlds/elastic_modulus_contact_test.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  EXPECT_TRUE(world != NULL);
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();

  int i = 0;
  while (!this->HasEntity("sphere") && i < 20)
  {
    common::Time::MSleep(100);
    ++i;
  }

  if (i > 20)
    gzthrow("Unable to get sphere");

  // get models and links
  physics::ModelPtr box_model = world->GetModel("box");
  physics::LinkPtr box_link = box_model->GetLink("link");
  physics::ModelPtr sphere_model = world->GetModel("sphere");
  physics::LinkPtr sphere_link = box_model->GetLink("link");

  // Sleep to ensure transport topics are all advertised
  common::Time::MSleep(100);
  std::list<std::string> topics =
    transport::getAdvertisedTopics("gazebo.msgs.Contacts");
  topics.sort();
  EXPECT_FALSE(topics.empty());
  EXPECT_EQ(topics.size(), 1u);

  auto topic = topics.front();

  gzdbg << "Listening to " << topic << std::endl;
  transport::SubscriberPtr sub = this->node->Subscribe(topic,
      &PhysicsTest::Callback, this);

  // step to let contact happen and settle
  world->Step(1000);

  // Wait for contact messages to be received
  int maxSleep = 30;
  int sleep = 0;
  while (this->contactsMsg.contact().size() < 1 && sleep < maxSleep)
  {
    common::Time::MSleep(100);
    sleep++;
  }
  ASSERT_EQ(this->contactsMsg.contact().size(), 1);
  // Copy message to local variable
  msgs::Contacts contacts;
  {
    boost::mutex::scoped_lock lock(this->mutex);
    contacts = this->contactsMsg;
  }

  // recompute stiffness by hand to double check physics
  // sphere
  const double nu1 = 0.4;
  const double E1 = 600000;
  const double m1 = 30;
  const double R1 = 0.06;
  // static box
  const double nu2 = 0.3;
  const double E2 = 500000;
  const double R2 = 0.08;
  double E_star = 1.0 / ((1.0 - nu1*nu1)/E1 + (1.0 - nu2*nu2)/E2);
  double R_star = 1.0 / (1.0/R1 + 1.0/R2);
  // contact force
  double f_g = -physics->GetGravity().z * m1;

  // get min contact depth from model
  const double minDepth = std::min(0.0015, 0.001);

  // get contact depth from contact manager
  ASSERT_EQ(contacts.contact().size(), 1);
  for (auto const &contact : contacts.contact())
  {
     gzerr << contact.collision1() << "\n";
     // gzerr << contact.collision2() << "\n";
     // gzerr << contact.depth() << "\n";
  }
  double d1 = 0;

  // get contact depth from link poses and known geometry information
  double d2 = 0;

  // contact patch radius
  double patchRadius = sqrt(R_star * d1);

  gzdbg << "f_g: " << f_g
        << "E: " << E_star
        << "curvature: " << R_star
        << "patch radius: " << patchRadius
        << "\n";

  // check contact information
}

TEST_P(PhysicsTest, ElasticModulusContact)
{
  ElasticModulusContact(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, PhysicsTest, PHYSICS_ENGINE_VALUES);

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

