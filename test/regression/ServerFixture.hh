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

               this->factoryPub =
                 this->node->Advertise<msgs::Factory>("~/factory");
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

  protected: void PrintImage(const std::string &_name, unsigned char **_image, 
                 unsigned int _width, unsigned int _height, unsigned int _depth)
             {
               unsigned int count = _height * _width * _depth;
               printf("\n");
               printf("static unsigned char __%s[] = {",_name.c_str());

               unsigned int i;
               for (i=0; i < count-1; i++)
               {
                 if (i % 10 == 0)
                   printf("\n");
                 printf("%d, ", (*_image)[i]);
               }
               printf("%d};\n", (*_image)[i]);
               printf("static unsigned char *%s = __%s;\n",_name.c_str(),
                   _name.c_str());
             }

  protected: void ImageCompare(unsigned char **_imageA, unsigned char *_imageB[],
                 unsigned int _width, unsigned int _height, unsigned int _depth,
                 unsigned int &_maxDiff, unsigned int &_diffSum,
                 double &_diffAvg)
             {
               _maxDiff = 0;
               _diffSum = 0;
               _diffAvg = 0;

               for (unsigned int y=0; y < _height; y++)
               {
                 for (unsigned int x=0; x < _width*_depth; x++)
                 {
                   unsigned int a = (*_imageA)[(y*_width*_depth)+x];
                   unsigned int b = (*_imageB)[(y*_width*_depth)+x];

                   unsigned int diff = (unsigned int)(fabs(a - b));

                   if (diff > _maxDiff)
                     _maxDiff = diff;

                   _diffSum += diff;
                 }
               }
               _diffAvg = _diffSum / (_height*_width*_depth);
             }

  private: void OnNewFrame(const unsigned char *_image,
                              unsigned int _width, unsigned int _height, 
                              unsigned int _depth,
                              const std::string &/*_format*/)
           {
             memcpy(*this->imgData, _image, _width * _height * _depth);
             this->gotImage+=1;
           }

  protected: void GetFrame(const std::string &_cameraName,
                 unsigned char **_imgData, unsigned int &_width,
                 unsigned int &_height)
             {
               sensors::SensorPtr sensor = sensors::get_sensor(_cameraName);
               EXPECT_TRUE(sensor);
               sensors::CameraSensorPtr camSensor =
                 boost::shared_dynamic_cast<sensors::CameraSensor>(sensor);

               _width = camSensor->GetImageWidth();
               _height = camSensor->GetImageHeight();

               if (*_imgData)
               {
                 delete *_imgData;
                 *_imgData = NULL;
               }
               (*_imgData) = new unsigned char[_width *_height*3];
               this->imgData = _imgData;

               this->gotImage = 0;
               event::ConnectionPtr c =
                 camSensor->GetCamera()->ConnectNewImageFrame(
                     boost::bind(&ServerFixture::OnNewFrame,
                                 this, _1, _2, _3, _4, _5));

               while (this->gotImage < 20)
                 usleep(1000);

               camSensor->GetCamera()->DisconnectNewImageFrame(c);
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
                 << "      <image width='320' height='240' format='R8G8B8'/>"
                 << "      <clip near='0.10000000000000001' far='100'/>"
                 //<< "      <save enabled='true' path='/tmp/camera/'/>"
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

  protected: Server *server;
  protected: boost::thread *serverThread;
  protected: transport::NodePtr node;
  protected: transport::SubscriberPtr poseSub;
  protected: std::map<std::string, math::Pose> poses;
  protected: boost::mutex *receiveMutex;

  private: unsigned char **imgData;
  private: int gotImage;
  protected: transport::PublisherPtr factoryPub;
};
