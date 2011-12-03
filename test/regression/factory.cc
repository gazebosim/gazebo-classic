#include "transport/TransportTypes.hh"
#include "transport/Node.hh"
#include "ServerFixture.hh"

using namespace gazebo;
class FactoryTest : public ServerFixture 
{
  protected: virtual ~FactoryTest()
             {
             }

  protected: virtual void Load(const std::string &_worldFilename)
             {
               ServerFixture::Load(_worldFilename);
               this->factoryPub =
                 this->node->Advertise<msgs::Factory>("~/factory");
             }

  protected: void SpawnCamera(const std::string &_modelName, 
                 const std::string &_cameraName,
                 const math::Vector3 &_pos, const math::Vector3 &_rpy)
             {
               msgs::Factory msg;
               std::ostringstream newModelStr;

               newModelStr << "<gazebo version='1.0'>"
                 << "<model name='" << _modelName << "' static='true'>"
                 << "<origin pose='" << _pos.x << " " 
                                     << _pos.y << " " 
                                     << _pos.z << " "
                                     << _rpy.x << " "
                                     << _rpy.y << " " 
                                     << _rpy.z << "'/>"
                 << "<link name='body'>"
                 << "  <sensor name='" << _cameraName << "' type='camera' always_on='1' update_rate='25' visualize='true'>"
                 << "    <camera>"
                 << "      <horizontal_fov angle='0.78539816339744828'/>"
                 << "      <image width='320' height='240' format='L8'/>"
                 << "      <clip near='0.10000000000000001' far='100'/>"
                 << "      <save enabled='true' path='/tmp/camera/'/>"
                 << "    </camera>"
                 << "  </sensor>"
                 << "</link>"
                 << "</model>"
                 << "</gazebo>";

               msg.set_sdf(newModelStr.str());
               this->factoryPub->Publish(msg);

               // Wait for the entity to spawn
               while (!this->HasEntity(_modelName))
                 usleep(10000);
             }
 
  protected: void SpawnCylinder(const std::string &_name, 
                 const math::Vector3 &_pos, const math::Vector3 &_rpy)
             {
               msgs::Factory msg;
               std::ostringstream newModelStr;

               newModelStr << "<gazebo version='1.0'>"
                 << "<model name='" << _name << "'>"
                 << "<origin pose='" << _pos.x << " " 
                                     << _pos.y << " " 
                                     << _pos.z << " "
                                     << _rpy.x << " "
                                     << _rpy.y << " " 
                                     << _rpy.z << "'/>"
                 << "<link name='body'>"
                 << "  <inertial mass='1.0'>"
                 << "    <inertia ixx='1' ixy='0' ixz='0' iyy='1'"
                 << " iyz='0' izz='1'/>"
                 << "  </inertial>"
                 << "  <collision name='geom'>"
                 << "    <geometry>"
                 << "      <cylinder radius='.5' length='1.0'/>"
                 << "    </geometry>"
                 << "  </collision>"
                 << "  <visual name='visual' cast_shadows='true'>"
                 << "    <geometry>"
                 << "      <cylinder radius='.5' length='1.0'/>"
                 << "    </geometry>"
                 << "  </visual>"
                 << "</link>"
                 << "</model>"
                 << "</gazebo>";

               msg.set_sdf(newModelStr.str());
               this->factoryPub->Publish(msg);

               // Wait for the entity to spawn
               while (!this->HasEntity(_name))
                 usleep(10000);
             }

  protected: void SpawnSphere(const std::string &_name, 
                 const math::Vector3 &_pos, const math::Vector3 &_rpy)
             {
               msgs::Factory msg;
               std::ostringstream newModelStr;

               newModelStr << "<gazebo version='1.0'>"
                 << "<model name='" << _name << "'>"
                 << "<origin pose='" << _pos.x << " " 
                                     << _pos.y << " " 
                                     << _pos.z << " "
                                     << _rpy.x << " "
                                     << _rpy.y << " " 
                                     << _rpy.z << "'/>"
                 << "<link name='body'>"
                 << "  <inertial mass='1.0'>"
                 << "    <inertia ixx='1' ixy='0' ixz='0' iyy='1'"
                 << " iyz='0' izz='1'/>"
                 << "  </inertial>"
                 << "  <collision name='geom'>"
                 << "    <geometry>"
                 << "      <sphere radius='.5'/>"
                 << "    </geometry>"
                 << "  </collision>"
                 << "  <visual name='visual' cast_shadows='true'>"
                 << "    <geometry>"
                 << "      <sphere radius='.5'/>"
                 << "    </geometry>"
                 << "  </visual>"
                 << "</link>"
                 << "</model>"
                 << "</gazebo>";

               msg.set_sdf(newModelStr.str());
               this->factoryPub->Publish(msg);

               // Wait for the entity to spawn
               while (!this->HasEntity(_name))
                 usleep(10000);
             }

  protected: void SpawnBox(const std::string &_name, 
                 const math::Vector3 &_pos, const math::Vector3 &_rpy)
             {
               msgs::Factory msg;
               std::ostringstream newModelStr;

               newModelStr << "<gazebo version='1.0'>"
                 << "<model name='" << _name << "'>"
                 << "<origin pose='" << _pos.x << " " 
                                     << _pos.y << " " 
                                     << _pos.z << " "
                                     << _rpy.x << " "
                                     << _rpy.y << " " 
                                     << _rpy.z << "'/>"
                 << "<link name='body'>"
                 << "  <inertial mass='1.0'>"
                 << "    <inertia ixx='1' ixy='0' ixz='0' iyy='1'"
                 << " iyz='0' izz='1'/>"
                 << "  </inertial>"
                 << "  <collision name='geom'>"
                 << "    <geometry>"
                 << "      <box size='1 1 1'/>"
                 << "    </geometry>"
                 << "  </collision>"
                 << "  <visual name='visual' cast_shadows='true'>"
                 << "    <geometry>"
                 << "      <box size='1 1 1'/>"
                 << "    </geometry>"
                 << "  </visual>"
                 << "</link>"
                 << "</model>"
                 << "</gazebo>";

               msg.set_sdf(newModelStr.str());
               this->factoryPub->Publish(msg);

               // Wait for the entity to spawn
               while (!this->HasEntity(_name))
                 usleep(10000);
             }

  protected: transport::PublisherPtr factoryPub;
};
/*
TEST_F(FactoryTest, Box)
{
  math::Pose setPose, testPose;
  Load("worlds/empty.world");

  for (unsigned int i=0; i < 100; i++)
  {
    std::ostringstream name;
    name << "test_box_" << i;
    setPose.Set(math::Vector3(0,0,i+0.5), math::Quaternion(0,0,0));
    SpawnBox(name.str(), setPose.pos, setPose.rot.GetAsEuler());
    testPose = GetEntityPose(name.str());
    testPose.Round(2);
    setPose.Round(2);
    EXPECT_EQ(testPose, setPose);
  }
}

TEST_F(FactoryTest, Sphere)
{
  math::Pose setPose, testPose;
  Load("worlds/empty.world");

  for (unsigned int i=0; i < 100; i++)
  {
    std::ostringstream name;
    name << "test_sphere_" << i;
    setPose.Set(math::Vector3(0,0,i+0.5), math::Quaternion(0,0,0));
    SpawnSphere(name.str(), setPose.pos, setPose.rot.GetAsEuler());
    testPose = GetEntityPose(name.str());
    testPose.Round(1);
    setPose.Round(1);
    EXPECT_EQ(testPose, setPose);
  }
}
*/

TEST_F(FactoryTest, Camera)
{
  math::Pose setPose, testPose;
  Load("worlds/empty.world");
  setPose.Set(math::Vector3(-5,0,5), math::Quaternion(0,DTOR(15),0));
  SpawnCamera("camera_model", "camera_sensor", setPose.pos,
      setPose.rot.GetAsEuler());
  usleep(1000000);
}


/*
TEST_F(FactoryTest, Cylinder)
{
  math::Pose setPose, testPose;
  Load("worlds/empty.world");

  for (unsigned int i=0; i < 100; i++)
  {
    std::ostringstream name;
    name << "test_cylinder_" << i;
    setPose.Set(math::Vector3(0,0,i+0.5), math::Quaternion(0,0,0));
    SpawnCylinder(name.str(), setPose.pos, setPose.rot.GetAsEuler());
    testPose = GetEntityPose(name.str());
    testPose.Round(1);
    setPose.Round(1);
    EXPECT_EQ(testPose, setPose);
  }
}
*/


int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
