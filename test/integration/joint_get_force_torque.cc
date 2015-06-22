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
#include <string.h>

#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/test/ServerFixture.hh"
#include "gazebo/test/helper_physics_generator.hh"

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
    public: SpawnGetFTBoxOptions() : mass(10.0),
                                     size(1.0, 1.0, 1.0),
                                     parentIsWorld(false)
            {
            }

    /// \brief Destructor.
    public: ~SpawnGetFTBoxOptions()
            {
            }

    /// \brief Mass of box to spawn (inertia computed automatically).
    public: double mass;

    /// \brief Size of box to spawn.
    public: math::Vector3 size;

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

    /// \brief If true the parent is a world, otherwise is a dummy link
    public: bool parentIsWorld;
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
              << "  <pose>" << _opt.modelPose << "</pose>";
              if ( !_opt.parentIsWorld )
              {
                modelStr
                  << "<link name='dummy_link'></link>";
              }
            modelStr
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
              << "  </link>";
              if ( !_opt.parentIsWorld )
              {
                modelStr
                  << "  <joint name='dummy_joint' type='fixed'>"
                  << "  <parent>world</parent>"
                  << "  <child>dummy_link</child>"
                  << "  <axis>"
                  << "    <limit>"
                  << "      <lower>0.0</lower>"
                  << "      <upper>0.0</upper>"
                  << "    </limit>"
                  << "    <xyz>" << _opt.jointAxis << "</xyz>"
                  << "  </axis>"
                  << "  </joint>";
              }
            modelStr
              << "  <joint name='joint' type='"
              << _opt.jointType << "'>";
              if ( _opt.parentIsWorld )
              {
                modelStr
                  << "    <parent>world</parent>";
              }
              else
              {
                modelStr
                  << "    <parent>dummy_link</parent>";
              }
            modelStr
              << "    <child>link</child>";
            // Remove this line when sdf will support
            // fixed joints without a dummy axis element
            // if( _opt.jointType != "fixed" )
            {
              modelStr
                << "  <axis>"
                << "    <limit>"
                << "      <lower>0.0</lower>"
                << "      <upper>0.0</upper>"
                << "    </limit>"
                << "    <xyz>" << _opt.jointAxis << "</xyz>"
                << "  </axis>";
            }
            modelStr
              << "<physics>"
              << "  <provide_feedback>true</provide_feedback>"
              << "</physics>"
              << "</joint>"
              << "</model>";

            physics::WorldPtr world = physics::get_world("default");
            world->InsertModelString(modelStr.str());

            physics::ModelPtr model;
            unsigned stepsToWait =  1000;

            unsigned int stepCount = 0;
            while (stepCount < stepsToWait &&
                   !this->HasEntity(modelName.str()))
            {
              world->Step(1);
              stepCount++;
              if (stepCount % 200 == 0)
              {
                gzwarn << "Waiting " << stepCount << " steps for "
                       << "box to spawn." << std::endl;
              }
            }

            if (this->HasEntity(modelName.str()))
              gzdbg << "box has spawned." << std::endl;

            if (world != NULL)
              model = world->GetModel(modelName.str());

            return model;
          }

  /// \brief Helper function for GetForceTorqueDemo
  /// \param[in] _world         Pointer to the world object
  /// \param[in] _physics       Pointer to the physics object.
  /// \param[in] _physicsEngine Physics engine to use.
  /// \param[in] _options       Options for the joint to test.
  public: void GetFTDemoHelper(physics::WorldPtr _world,
                               physics::PhysicsEnginePtr _physics,
                               const std::string &_physicsEngine,
                               const SpawnGetFTBoxOptions & opt);

  /// \brief Test GetForceTorque method for different type joints
  /// \param[in] _physicsEngine Physics engine to use.
  public: void GetForceTorqueDemo(const std::string &_physicsEngine);

  /// \brief Count of spawned models, used to ensure unique model names.
  private: unsigned int spawnCount;
};

void JointGetForceTorqueTest::GetFTDemoHelper(
                                           physics::WorldPtr _world,
                                           physics::PhysicsEnginePtr _physics,
                                           const std::string& _physicsEngine,
                                           const SpawnGetFTBoxOptions& opt)
{
  gzdbg << "GetFTDemoHelper for physics " << _physicsEngine
        << "joint type " << opt.jointType
        << " joint axis " << opt.jointAxis << std::endl;
  math::Vector3 g = _physics->GetGravity();
  double mass = opt.mass;

  physics::ModelPtr model = SpawnBox(opt);
  physics::LinkPtr  link  = model->GetLink("link");
  physics::JointPtr joint = model->GetJoint("joint");
  math::Vector3 com = link->GetWorldCoGPose().pos;
  math::Vector3 jointOrigin = joint->GetWorldPose().pos;

  ASSERT_TRUE(model != NULL);
  ASSERT_TRUE(link != NULL);

  // do a simulation step to get a meaningful measure
  _world->Step(1);

  // ode need some additional steps
  if ( _physicsEngine == "ode" )
  {
      _world->Step(9);
  }

  // bullet need some additional steps
  // probably related to
  if ( _physicsEngine == "bullet" )
  {
      _world->Step(99);
  }

  if ( opt.parentIsWorld )
  {
      ASSERT_EQ(model->GetJointCount(), 1);
  }
  else
  {
      ASSERT_EQ(model->GetJointCount(), 2);
  }

  ASSERT_TRUE(joint != NULL);

  physics::JointWrench W = joint->GetForceTorque(0u);

  const int TOL_FORCE = 1.0;
  const int TOL_TORQUE = 2.0;

  // Everthing is expressed in world frame, so
  // the math is easy
  math::Vector3 fWorld = mass*g;
  math::Vector3 tauWorld = mass*(com-jointOrigin).Cross(g);

  math::Pose parentPose;
  math::Pose childPose = link->GetWorldPose();

  if ( !opt.parentIsWorld )
  {
      parentPose = link->GetParentJointsLinks()[0]->GetWorldPose();
  }

  math::Vector3 body1ForceExpected  = -(parentPose.rot.GetInverse()*fWorld);
  math::Vector3 body1TorqueExpected = -(parentPose.rot.GetInverse()*tauWorld);
  math::Vector3 body2ForceExpected  =   childPose.rot.GetInverse()*fWorld;
  math::Vector3 body2TorqueExpected =   childPose.rot.GetInverse()*tauWorld;

  EXPECT_NEAR(body1ForceExpected.x, W.body1Force.x, TOL_FORCE);
  EXPECT_NEAR(body1ForceExpected.y, W.body1Force.y, TOL_FORCE);
  EXPECT_NEAR(body1ForceExpected.z, W.body1Force.z, TOL_FORCE);
  EXPECT_NEAR(body1TorqueExpected.x, W.body1Torque.x, TOL_TORQUE);
  EXPECT_NEAR(body1TorqueExpected.y, W.body1Torque.y, TOL_TORQUE);
  EXPECT_NEAR(body1TorqueExpected.z, W.body1Torque.z, TOL_TORQUE);

  EXPECT_NEAR(body2ForceExpected.x, W.body2Force.x, TOL_FORCE);
  EXPECT_NEAR(body2ForceExpected.y, W.body2Force.y, TOL_FORCE);
  EXPECT_NEAR(body2ForceExpected.z, W.body2Force.z, TOL_FORCE);
  EXPECT_NEAR(body2TorqueExpected.x, W.body2Torque.x, TOL_TORQUE);
  EXPECT_NEAR(body2TorqueExpected.y, W.body2Torque.y, TOL_TORQUE);
  EXPECT_NEAR(body2TorqueExpected.z, W.body2Torque.z, TOL_TORQUE);

  // Remove model
  _world->RemoveModel(model);
}



/////////////////////////////////////////////////
// GetForceTorqueDemo test:
// spawn a simple box connected to the world
// with different types of "fixed" joints
// (actual fixed joint, or revolute/prismatic joint
//  with zero limis) and then check the GetForceTorque
// output against analytical solution.
void JointGetForceTorqueTest::GetForceTorqueDemo(const std::string &_physEng)
{
  Load("worlds/empty.world", true, _physEng);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // check the gravity vector
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physEng);

  // test for a fixed joint
  SpawnGetFTBoxOptions opt;
  opt.jointType = "fixed";
  opt.inertialPose.pos = math::Vector3(1.0, 2.0, 3.0);
  opt.linkPose.rot.SetFromEuler(0.1, 0.0, 0.0);

  opt.parentIsWorld = true;
  GetFTDemoHelper(world, physics, _physEng, opt);

  opt.parentIsWorld = false;
  GetFTDemoHelper(world, physics, _physEng, opt);

  // test a revolute joint against all axis

  if ( _physEng == "bullet" )
  {
    for (int i = 0; i < 3; i++)
    {
        opt.jointType = "revolute";
        switch ( i )
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

        GetFTDemoHelper(world, physics, _physEng, opt);
    }
  }

  // test a prismatic joint against all axis

  // bullet and simbody GetForceTorque() is
  // broken for prismatic joints
  // see gazebo issues 1639 and 1640
  if ( _physEng != "bullet" &&
       _physEng != "simbody" )
  {
    for (int i = 0; i < 3; i++)
    {
      opt.jointType = "prismatic";
      switch ( i )
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
      GetFTDemoHelper(world, physics, _physEng, opt);
    }
  }
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
