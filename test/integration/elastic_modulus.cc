/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#include <math.h>

#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/test/helper_physics_generator.hh>
#include <gazebo/test/ServerFixture.hh>
#include <gazebo/transport/transport.hh>

#include <ignition/math/Rand.hh>

#define PHYSICS_TOL 0.008

using namespace gazebo;

class PhysicsTest : public ServerFixture,
                    public testing::WithParamInterface<const char*>
{
  /// \brief test elastic modulus
  public: void ElasticModulusContact(const std::string &_physicsEngine);

  /// \brief Callback for contact subscribers in depth test.
  /// \param[in] _msg Contact message
  private: void ContactCallback(const ConstContactsPtr &_msg);

  /// \brief Message to be filled with the latest contacts message.
  private: msgs::Contacts contactsMsg;

  /// \brief Mutex to protect reads and writes to contactsMsg.
  public: mutable boost::mutex mutex;
};

/////////////////////////////////////////////////
void PhysicsTest::ContactCallback(const ConstContactsPtr &_msg)
{
  boost::mutex::scoped_lock lock(this->mutex);
  this->contactsMsg = *_msg;
}

////////////////////////////////////////////////////////////////////////
void PhysicsTest::ElasticModulusContact(const std::string &_physicsEngine)
{
  if (_physicsEngine != "ode")
  {
    gzerr << "Elastic Modulus is only implemented for ODE.\n";
    return;
  }

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
  physics::LinkPtr sphere_link = sphere_model->GetLink("link");

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
      &PhysicsTest::ContactCallback, this);

  // step to let contact happen and settle
  world->Step(3000);

  // Wait for contact messages to be received
  int maxSleep = 100;
  int sleep = 0;
  while (this->contactsMsg.contact().size() < 1 && sleep < maxSleep)
  {
    common::Time::MSleep(100);
    sleep++;
  }
  ASSERT_EQ(this->contactsMsg.contact().size(), 1);

  // recorded from opende/src/joints/contact.cpp:208
  const double d_converged = 0.0258629;
  const double k_converged = 71454.617;
  const double k_lin_converged = 11491.286;

  // sphere
  const double nu1 = 0.4;
  const double e1 = 600000;
  const double m1 = 3;
  const double r1 = 0.06;
  // static box
  const double nu2 = 0.3;
  const double e2 = 500000;
  const double r2 = 0.08;
  double eStar = 1.0 / ((1.0 - nu1*nu1)/e1 + (1.0 - nu2*nu2)/e2);
  double rStar = 1.0 / (1.0/r1 + 1.0/r2);
  // contact force
  double f1 = -physics->GetGravity().x * m1;

  for (int n = 0; n < 10; ++n)
  {
    world->Step(1);
    // Copy message to local variable
    msgs::Contacts contacts;
    {
      boost::mutex::scoped_lock lock(this->mutex);
      contacts = this->contactsMsg;
    }

    // recompute stiffness by hand to double check physics

    // get min contact depth from model
    const double minDepth = std::min(0.0015, 0.001);

    // GET CONTACT DEPTH FROM CONTACT MANAGER
    ASSERT_EQ(contacts.contact().size(), 1);
    double d1 = 0;
    for (auto const &contact : contacts.contact())
    {
      gzdbg << "col1 [" << contact.collision1()
            << "] col2 [" << contact.collision2()
            << "]\n";
      EXPECT_EQ(contact.depth().size(), 1);
      for (auto const &d : contact.depth())
      {
        d1 = d - minDepth;
        // gzerr << d1 << "\n";
      }
    }
    double d_convergence_error = (d1 - d_converged)/d_converged;
    EXPECT_LT(d1, d_converged);
    EXPECT_LT(fabs(d_convergence_error), PHYSICS_TOL);

    // GET CONTACT DEPTH FROM LINK POSES AND KNOWN GEOMETRY INFORMATION
    double d2 = 1.0 - (sphere_link->GetWorldPose().pos.x -
                 box_link->GetWorldPose().pos.x) - minDepth;

    EXPECT_FLOAT_EQ(d1, d2);

    // GET CONTACT DEPTH BASED ON CONTACT MODEL
    // k = f / d^1.5 or f = k * d^1.5.
    // And we know that:
    //   k = 4.0 / 3.0 * e_star * sqrt(patch_radius);
    // or
    //   k = 4.0 / 3.0 * e_star * sqrt(sqrt(rStar * depth));
    // and
    //   f = k * depth^1.5
    // therefore
    //   f = 4.0 / 3.0 * e_star * sqrt(sqrt(rStar)) * depth^1.75
    // knowing f,
    // solving for d yeilds analytical depth:
    double d3 = pow(f1 / (4.0 / 3.0 * eStar * sqrt(sqrt(rStar))), 1.0/1.75);

    // check that d1 and d3 should be near
    EXPECT_LT(fabs(d1-d3)/d3, PHYSICS_TOL);

    // linearized contact patch radius
    double patchRadius = sqrt(rStar * d1);

    // check stiffness
    double k = 4.0 / 3.0 * eStar * sqrt(patchRadius);
    // check linearized stiffness (should match ode internal k_hertz_sqrtx)
    double k_lin = 4.0 / 3.0 * eStar * sqrt(patchRadius*d1);

    // recorded from opende/src/joints/contact.cpp:208
    EXPECT_FLOAT_EQ(k,     k_converged);
    // recorded from opende/src/joints/contact.cpp:208
    EXPECT_FLOAT_EQ(k_lin, k_lin_converged);

    gzdbg << "Contact State:\n"
          << "  t [" << world->GetSimTime().Double()
          << "]\n f1 [" << f1
          << "]\n  E* [" << eStar
          << "]\n  R* [" << rStar
          << "]\n  patch radius [" << patchRadius
          << "]\n  d contact manager [" << d1
          << "]\n  d convergence error [" << d_convergence_error
          << "]\n  d geom [" << d2
          << "]\n  d solution [" << d3
          << "]\n  d relative error [" << (d1-d3)/d3
          << "]\n  patch radius [" << patchRadius
          << "]\n  k(Hertzian) [" << k
          << "]\n  k relative error [" << (k - k_converged)/k_converged
          << "]\n  k(liearized about depth) [" << k_lin
          << "]\n  k lin relative error ["
          << (k_lin - k_lin_converged)/k_lin_converged
          << "] \n";
  }
}

/////////////////////////////////////////////////
TEST_P(PhysicsTest, ElasticModulusContact)
{
  ElasticModulusContact(GetParam());
}

/////////////////////////////////////////////////
INSTANTIATE_TEST_CASE_P(PhysicsEngines, PhysicsTest, PHYSICS_ENGINE_VALUES);

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

