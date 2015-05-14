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
#include <string.h>

#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/test/ServerFixture.hh"
#include "helper_physics_generator.hh"

using namespace gazebo;

const double g_friction_tolerance = 1e-3;

class JointGetForceTorqueTest : public ServerFixture,
                        public testing::WithParamInterface<const char*>
{
  protected: JointGetForceTorqueTest() : ServerFixture(), spawnCount(0)
             {
             }



  /// \brief Class to hold parameters for spawning joints.
  public: class SpawnGetFTBoxOptions
  {
    /// \brief Constructor.
    public: SpawnGetFTBoxOptions() : mass(10.0)
            {
            }

    /// \brief Destructor.
    public: ~SpawnGetFTBoxOptions()
            {
            }

    /// \brief Size of box to spawn.
    public: math::Vector3 size;

    /// \brief Mass of box to spawn (inertia computed automatically).
    public: double mass;

    /// \brief Model pose.
    public: math::Pose modelPose;

    /// \brief Link pose.
    public: math::Pose linkPose;

    /// \brief Inertial pose.
    public: math::Pose inertialPose;

    /// \brief Joint type
    public: std::string jointType;

    /// \brief Joint axis
    public: math::Vector3 jointAxis;

  };

  /// \brief Spawn a box with friction coefficients and direction.
  /// \param[in] _opt Options for friction box.
  public: physics::ModelPtr SpawnBox(const SpawnGetFTBoxOptions &_opt)
          {
            msgs::Factory msg;
            std::ostringstream modelStr;
            std::ostringstream modelName;
            modelName << "box_model" << this->spawnCount++;

            double dx = _opt.size.x;
            double dy = _opt.size.y;
            double dz = _opt.size.z;
            double ixx = _opt.mass/12.0 * (dy*dy + dz*dz);
            double iyy = _opt.mass/12.0 * (dz*dz + dx*dx);
            double izz = _opt.mass/12.0 * (dx*dx + dy*dy);

            modelStr
              << "<sdf version='" << SDF_VERSION << "'>"
              << "<model name ='" << modelName.str() << "'>"
              << "  <pose>" << _opt.modelPose << "</pose>"
              << "  <link name='link'>"
              << "    <pose>" << _opt.linkPose << "</pose>"
              << "    <inertial>"
              << "      <pose>" << _opt.inertialPose << "</pose>"
              << "      <mass>" << _opt.mass << "</mass>"
              << "      <inertia>"
              << "        <ixx>" << ixx << "</ixx>"
              << "        <iyy>" << iyy << "</iyy>"
              << "        <izz>" << izz << "</izz>"
              << "        <ixy>" << 0.0 << "</ixy>"
              << "        <ixz>" << 0.0 << "</ixz>"
              << "        <iyz>" << 0.0 << "</iyz>"
              << "      </inertia>"
              << "    </inertial>"
//              << "    <collision name='collision'>"
//              << "      <pose>" << _opt.collisionPose << "</pose>"
//              << "      <geometry>"
//              << "        <box><size>" << _opt.size << "</size></box>"
//              << "      </geometry>"
//              << "    </collision>"
//              << "    <visual name='visual'>"
//              << "      <pose>" << _opt.collisionPose << "</pose>"
//              << "      <geometry>"
//              << "        <box><size>" << _opt.size << "</size></box>"
//              << "      </geometry>"
//              << "    </visual>"
              << "  </link>"
              << "  <joint name='joint' type='"
              << _opt.jointType << "'>"
              << "  <parent>world</parent>"
              << "  <child>link</child>";
            //if( _opt.jointType != "fixed" )
            {
              modelStr
                << "<axis>"
                << "  <limit>"
                << "    <lower>0.0</lower>"
                << "    <upper>0.0</upper>"
                << "  </limit>"
                << "  <xyz>" << _opt.jointAxis << "</xyz>"
                << "</axis>";
            }
            modelStr
              << "  </joint>"
              << "</model>";

            physics::WorldPtr world = physics::get_world("default");
            world->InsertModelString(modelStr.str());

            physics::ModelPtr model;
            common::Time wait(100, 0);

            common::Time wallStart = common::Time::GetWallTime();
            unsigned int waitCount = 0;
            while (wait > (common::Time::GetWallTime() - wallStart) &&
                   !this->HasEntity(modelName.str()))
            {
              common::Time::MSleep(10);
              if (++waitCount % 100 == 0)
              {
                gzwarn << "Waiting " << waitCount / 100 << " seconds for "
                       << "box to spawn." << std::endl;
              }
            }
            if (this->HasEntity(modelName.str()) && waitCount >= 100)
              gzwarn << "box has spawned." << std::endl;

            if (world != NULL)
              model = world->GetModel(modelName.str());

            return model;
          }

  /// \brief Helper function for GetForceTorqueDemo
  /// \param[in] _physics       Pointer to the physics object.
  /// \param[in] _physicsEngine Physics engine to use.
  /// \param[in] _options       Options for the joint to test.
  public: void GetFTDemoHelper(physics::PhysicsEnginePtr _physics,
                               const std::string &_physicsEngine,
                               const SpawnGetFTBoxOptions & opt);

  /// \brief Test GetForceTorque method for different type joints
  /// \param[in] _physicsEngine Physics engine to use.
  public: void GetForceTorqueDemo(const std::string &_physicsEngine);

  /// \brief Count of spawned models, used to ensure unique model names.
  private: unsigned int spawnCount;
};

void JointGetForceTorqueTest::GetFTDemoHelper(
                                           physics::PhysicsEnginePtr _physics,
                                           const std::string& /*_physicsEngine*/,
                                           const SpawnGetFTBoxOptions& opt)
{
    math::Vector3 g = _physics->GetGravity();
    double mass = opt.mass;
    math::Vector3 com = opt.inertialPose.pos;

    physics::ModelPtr model = SpawnBox(opt);

    ASSERT_TRUE(model->GetJointCount() == 1);

    physics::JointPtr joint = model->GetJoint("joint");

    ASSERT_TRUE(joint != NULL);

    // wait some time to get a clean measure
    common::Time::MSleep(100);

    physics::JointWrench W = joint->GetForceTorque(0u);

    const int TOL_FORCE = 1.0;
    const int TOL_TORQUE = 2.0;

    // Everthing is expressed in world frame, so
    // the math is easy
    math::Vector3 f = mass*g;
    math::Vector3 tau = mass*com.Cross(g);

    EXPECT_NEAR(-f.x,W.body1Force.x,TOL_FORCE);
    EXPECT_NEAR(-f.y,W.body1Force.y,TOL_FORCE);
    EXPECT_NEAR(-f.z,W.body1Force.z,TOL_FORCE);
    EXPECT_NEAR(-tau.x,W.body1Torque.x,TOL_TORQUE);
    EXPECT_NEAR(-tau.y,W.body1Torque.y,TOL_TORQUE);
    EXPECT_NEAR(-tau.z,W.body1Torque.z,TOL_TORQUE);

    EXPECT_NEAR(f.x,W.body2Force.x,TOL_FORCE);
    EXPECT_NEAR(f.y,W.body2Force.y,TOL_FORCE);
    EXPECT_NEAR(f.z,W.body2Force.z,TOL_FORCE);
    EXPECT_NEAR(tau.x,W.body2Torque.x,TOL_TORQUE);
    EXPECT_NEAR(tau.y,W.body2Torque.y,TOL_TORQUE);
    EXPECT_NEAR(tau.z,W.body2Torque.z,TOL_TORQUE);
}



/////////////////////////////////////////////////
// GetForceTorqueDemo test:
// spawn a simple box connected to the world
// with different types of "fixed" joints
// (actual fixed joint, or revolute/prismatic joint
//  with zero limis) and then check the GetForceTorque
// output against analytical solution.
void JointGetForceTorqueTest::GetForceTorqueDemo(const std::string &_physicsEngine)
{
  Load("worlds/empty.world", false, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // check the gravity vector
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  // test for a fixed joint
  SpawnGetFTBoxOptions opt;
  opt.jointType = "fixed";

  if( _physicsEngine != "simbody" )
  {
    gzerr << "Aborting test since fixed joints are implemented only"
          << " on simbody"
          << std::endl;
    return;
  }

  GetFTDemoHelper(physics,_physicsEngine,opt);

  // test a revolute joint against all axis
  /*
  for(int i=0; i < 3; i++)
  {
    opt.jointType = "revolute";

    switch( i )
    {
    case 0:
        opt.jointAxis = math::Vector3::UnitX;
        break;
    case 1:
        opt.jointAxis = math::Vector3::UnitY;
        break;
    case 2:
        opt.jointAxis = math::Vector3::UnitZ;
        break;
    }

    GetFTDemoHelper(physics,_physicsEngine,opt);
  }
  */
  // test a prismatic joint against all axis
  /*
  for(int i=0; i < 3; i++)
  {
    opt.jointType = "prismatic";

    switch( i )
    {
    case 0:
        opt.jointAxis = math::Vector3::UnitX;
        break;
    case 1:
        opt.jointAxis = math::Vector3::UnitY;
        break;
    case 2:
        opt.jointAxis = math::Vector3::UnitZ;
        break;
    }


    GetFTDemoHelper(physics,_physicsEngine,opt);
  }
  */
}

/////////////////////////////////////////////////
TEST_P(JointGetForceTorqueTest, GetForceTorqueDemo)
{
  GetForceTorqueDemo(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, JointGetForceTorqueTest,
                        PHYSICS_ENGINE_VALUES);

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
