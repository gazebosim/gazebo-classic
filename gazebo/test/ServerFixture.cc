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
#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
#include <Winsock2.h>
#endif

#include <stdio.h>
#include <string>
#include <cmath>
#include <boost/algorithm/string.hpp>

#include "gazebo/gazebo.hh"
#include "ServerFixture.hh"

using namespace gazebo;

/////////////////////////////////////////////////
std::string gazebo::custom_exec(std::string _cmd)
{
  _cmd += " 2>/dev/null";

#ifdef _WIN32
  FILE *pipe = _popen(_cmd.c_str(), "r");
#else
  FILE *pipe = popen(_cmd.c_str(), "r");
#endif

  if (!pipe)
    return "ERROR";

  char buffer[128];
  std::string result = "";

  while (!feof(pipe))
  {
    if (fgets(buffer, 128, pipe) != NULL)
      result += buffer;
  }

#ifdef _WIN32
  _pclose(pipe);
#else
  pclose(pipe);
#endif

  return result;
}

/////////////////////////////////////////////////
ServerFixture::ServerFixture()
{
  this->server = NULL;
  this->serverRunning = false;
  this->paused = false;
  this->percentRealTime = 0;
  this->gotImage = 0;
  this->imgData = NULL;
  this->serverThread = NULL;
  this->uniqueCounter = 0;

  gzLogInit("test-", "test.log");
  gazebo::common::Console::SetQuiet(false);
  common::SystemPaths::Instance()->AddGazeboPaths(
      TEST_INTEGRATION_PATH);

  // Add local search paths
  boost::filesystem::path path;

  path = PROJECT_SOURCE_PATH;
  gazebo::common::SystemPaths::Instance()->AddGazeboPaths(path.string());

  path = PROJECT_SOURCE_PATH;
  path /= "gazebo";
  gazebo::common::SystemPaths::Instance()->AddGazeboPaths(path.string());

  path = PROJECT_BINARY_PATH;
  path /= "plugins";
  gazebo::common::SystemPaths::Instance()->AddPluginPaths(path.string());

  path = PROJECT_BINARY_PATH;
  path /= "test";
  path /= "plugins";
  gazebo::common::SystemPaths::Instance()->AddPluginPaths(path.string());

  path = TEST_PATH;
  gazebo::common::SystemPaths::Instance()->AddGazeboPaths(path.string());
}

/////////////////////////////////////////////////
ServerFixture::~ServerFixture()
{
}

/////////////////////////////////////////////////
void ServerFixture::TearDown()
{
  this->Unload();
}

/////////////////////////////////////////////////
void ServerFixture::Unload()
{
  gzdbg << "ServerFixture::Unload" << std::endl;
  this->serverRunning = false;
  if (this->node)
    this->node->Fini();

  if (this->server)
  {
    this->server->Stop();

    if (this->serverThread)
    {
      this->serverThread->join();
    }
  }

  delete this->serverThread;
  this->serverThread = NULL;
}

/////////////////////////////////////////////////
void ServerFixture::Load(const std::string &_worldFilename)
{
  this->Load(_worldFilename, false);
}

/////////////////////////////////////////////////
void ServerFixture::Load(const std::string &_worldFilename, bool _paused)
{
  std::string s("");
  this->Load(_worldFilename, _paused, s);
}

/////////////////////////////////////////////////
void ServerFixture::Load(const std::string &_worldFilename,
                         bool _paused, const std::string &_physics,
                         const std::vector<std::string> &_systemPlugins)
{
  std::string params = _worldFilename;
  if (!_physics.empty())
    params += " -e " + _physics;
  if (_paused)
    params += " -u";
  for (auto plugin : _systemPlugins)
    params += " -s " + plugin;

  this->LoadArgs(params);
}

/////////////////////////////////////////////////
void ServerFixture::LoadArgs(const std::string &_args)
{
  delete this->server;
  this->server = NULL;

  // Split the string into a vector of parameters.
  std::vector<std::string> params;
  std::string args = _args;
  boost::trim_if(args, boost::is_any_of("\t "));
  boost::split(params, args, boost::is_any_of("\t "), boost::token_compress_on);

  bool paused = false;
  if (std::find(params.begin(), params.end(), "-u") != params.end())
    paused = true;

  // Create, load, and run the server in its own thread
  this->serverThread = new boost::thread(
    boost::bind(&ServerFixture::RunServer, this, params));

  // Wait for the server to come up
  // Use a 60 second timeout.
  int waitCount = 0, maxWaitCount = 6000;
  while ((!this->server || !this->server->GetInitialized()) &&
         ++waitCount < maxWaitCount)
    common::Time::MSleep(100);
  gzdbg << "ServerFixture load in "
        << static_cast<double>(waitCount)/10.0
        << " seconds, timeout after "
        << static_cast<double>(maxWaitCount)/10.0
        << " seconds\n";

  if (waitCount >= maxWaitCount)
    this->launchTimeoutFailure("while waiting for Load() function", waitCount);

  this->node = transport::NodePtr(new transport::Node());
  ASSERT_NO_THROW(this->node->Init());
  this->poseSub = this->node->Subscribe("~/pose/local/info",
      &ServerFixture::OnPose, this, true);
  this->statsSub = this->node->Subscribe("~/world_stats",
      &ServerFixture::OnStats, this);

  this->factoryPub =
    this->node->Advertise<msgs::Factory>("~/factory");

  this->requestPub =
    this->node->Advertise<msgs::Request>("~/request");

  // Wait for the world to reach the correct pause state.
  // This might not work properly with multiple worlds.
  // Use a 30 second timeout.
  waitCount = 0;
  maxWaitCount = 3000;
  while ((!physics::get_world() ||
           physics::get_world()->IsPaused() != paused) &&
         ++waitCount < maxWaitCount)
  {
    common::Time::MSleep(100);
  }

  ASSERT_LT(waitCount, maxWaitCount);

  this->factoryPub->WaitForConnection();
  this->requestPub->WaitForConnection();
}

/////////////////////////////////////////////////
void ServerFixture::RunServer(const std::vector<std::string> &_args)
{
  // Make room for an extra parameter (gzserver).
  int argc = _args.size() + 1;
  char **argv = new char* [argc];

  // The first parameter is the name of the program.
  const char *cmd = "gzserver";
  argv[0] = strdup(cmd);

  // Copy the command line parameters for gzserver.
  for (size_t i = 0; i < _args.size(); ++i)
    argv[i + 1] = strdup(_args.at(i).c_str());

  ASSERT_NO_THROW(this->server = new Server());

  if (!this->server->ParseArgs(argc, argv))
  {
    ASSERT_NO_THROW(delete this->server);
    this->server = NULL;
    return;
  }

  if (!rendering::get_scene(gazebo::physics::get_world()->GetName()))
  {
    ASSERT_NO_THROW(rendering::create_scene(
        gazebo::physics::get_world()->GetName(), false, true));
  }

  ASSERT_NO_THROW(this->server->Run());

  ASSERT_NO_THROW(this->server->Fini());

  ASSERT_NO_THROW(delete this->server);
  this->server = NULL;

  // Deallocate memory for the command line arguments allocated with strdup.
  for (int i = 0; i < argc; ++i)
    free(argv[i]);

  delete[] argv;
}

/////////////////////////////////////////////////
rendering::ScenePtr ServerFixture::GetScene(
    const std::string &_sceneName)
{
  // Wait for the scene to get loaded.
  int i = 0;
  int timeoutDS = 20;
  while (rendering::get_scene(_sceneName) == NULL && i < timeoutDS)
  {
    common::Time::MSleep(100);
    ++i;
  }

  if (i >= timeoutDS)
  {
    gzerr << "Unable to load the rendering scene.\n"
          << "Test will fail";
    this->launchTimeoutFailure(
        "while waiting to load rendering scene", i);
  }

  return rendering::get_scene(_sceneName);
}


/////////////////////////////////////////////////
void ServerFixture::OnStats(ConstWorldStatisticsPtr &_msg)
{
  this->simTime = msgs::Convert(_msg->sim_time());
  this->realTime = msgs::Convert(_msg->real_time());
  this->pauseTime = msgs::Convert(_msg->pause_time());
  this->paused = _msg->paused();

  if (this->realTime == 0)
    this->percentRealTime = 0;
  else
    this->percentRealTime =
      (this->simTime / this->realTime).Double();

  this->serverRunning = true;
}

/////////////////////////////////////////////////
void ServerFixture::SetPause(bool _pause)
{
  physics::pause_worlds(_pause);
}

/////////////////////////////////////////////////
double ServerFixture::GetPercentRealTime() const
{
  while (!this->serverRunning)
    common::Time::MSleep(100);

  return this->percentRealTime;
}

/////////////////////////////////////////////////
void ServerFixture::OnPose(ConstPosesStampedPtr &_msg)
{
  boost::mutex::scoped_lock lock(this->receiveMutex);
  for (int i = 0; i < _msg->pose_size(); ++i)
  {
    this->poses[_msg->pose(i).name()] = msgs::ConvertIgn(_msg->pose(i));
  }
}

/////////////////////////////////////////////////
math::Pose ServerFixture::GetEntityPose(const std::string &_name)
{
  boost::mutex::scoped_lock lock(this->receiveMutex);

  std::map<std::string, math::Pose>::iterator iter;
  iter = this->poses.find(_name);
  EXPECT_TRUE(iter != this->poses.end());
  return iter->second;
}

/////////////////////////////////////////////////
bool ServerFixture::HasEntity(const std::string &_name)
{
  boost::mutex::scoped_lock lock(this->receiveMutex);
  std::map<std::string, math::Pose>::iterator iter;
  iter = this->poses.find(_name);
  return iter != this->poses.end();
}

/////////////////////////////////////////////////
void ServerFixture::PrintImage(const std::string &_name, unsigned char **_image,
    unsigned int _width, unsigned int _height, unsigned int _depth)
{
  unsigned int count = _height * _width * _depth;
  printf("\n");
  printf("static unsigned char __%s[] = {", _name.c_str());
  unsigned int i;
  for (i = 0; i < count-1; i++)
  {
    if (i % 10 == 0)
      printf("\n");
    else
      printf(" ");
    printf("%d,", (*_image)[i]);
  }
  printf(" %d};\n", (*_image)[i]);
  printf("static unsigned char *%s = __%s;\n", _name.c_str(),
      _name.c_str());
}

/////////////////////////////////////////////////
void ServerFixture::PrintScan(const std::string &_name, double *_scan,
               unsigned int _sampleCount)
{
  printf("static double __%s[] = {\n", _name.c_str());
  for (unsigned int i = 0; i < _sampleCount-1; ++i)
  {
    if ((i+1) % 5 == 0)
      printf("%13.10f,\n", math::precision(_scan[i], 10));
    else
      printf("%13.10f, ", math::precision(_scan[i], 10));
  }
  printf("%13.10f};\n",
      math::precision(_scan[_sampleCount-1], 10));
  printf("static double *%s = __%s;\n", _name.c_str(),
      _name.c_str());
}

/////////////////////////////////////////////////
void ServerFixture::FloatCompare(float *_scanA, float *_scanB,
    unsigned int _sampleCount, float &_diffMax,
    float &_diffSum, float &_diffAvg)
{
  _diffMax = 0;
  _diffSum = 0;
  _diffAvg = 0;
  for (unsigned int i = 0; i < _sampleCount; ++i)
  {
    double diff = fabs(math::precision(_scanA[i], 10) -
                math::precision(_scanB[i], 10));
    _diffSum += diff;
    if (diff > _diffMax)
    {
      _diffMax = diff;
    }
  }
  _diffAvg = _diffSum / _sampleCount;
}

/////////////////////////////////////////////////
void ServerFixture::DoubleCompare(double *_scanA, double *_scanB,
    unsigned int _sampleCount, double &_diffMax,
    double &_diffSum, double &_diffAvg)
{
  _diffMax = 0;
  _diffSum = 0;
  _diffAvg = 0;
  for (unsigned int i = 0; i < _sampleCount; ++i)
  {
    double diff;

    // set diff = 0 if both values are same-sign infinite, as inf - inf = nan
    if (std::isinf(_scanA[i]) && std::isinf(_scanB[i]) &&
      _scanA[i] * _scanB[i] > 0)
    {
      diff = 0;
    }
    else
    {
      diff = fabs(math::precision(_scanA[i], 10) -
                math::precision(_scanB[i], 10));
    }

    _diffSum += diff;
    if (diff > _diffMax)
    {
      _diffMax = diff;
    }
  }
  _diffAvg = _diffSum / _sampleCount;
}

/////////////////////////////////////////////////
void ServerFixture::ImageCompare(unsigned char *_imageA,
    unsigned char *_imageB,
    unsigned int _width, unsigned int _height, unsigned int _depth,
    unsigned int &_diffMax, unsigned int &_diffSum,
    double &_diffAvg)
{
  _diffMax = 0;
  _diffSum = 0;

  for (unsigned int y = 0; y < _height; ++y)
  {
    for (unsigned int x = 0; x < _width*_depth; ++x)
    {
      int a = _imageA[(y*_width*_depth)+x];
      int b = _imageB[(y*_width*_depth)+x];

      unsigned int absDiff = abs(a - b);

      if (absDiff > _diffMax)
        _diffMax = absDiff;

      _diffSum += absDiff;
    }
  }
  _diffAvg = _diffSum / (_height*_width*_depth);
}

/////////////////////////////////////////////////
void ServerFixture::OnNewFrame(const unsigned char *_image,
                 unsigned int _width, unsigned int _height,
                 unsigned int _depth,
                 const std::string &/*_format*/)
{
  memcpy(*this->imgData, _image, _width * _height * _depth);
  this->gotImage+= 1;
}

/////////////////////////////////////////////////
void ServerFixture::GetFrame(const std::string &_cameraName,
    unsigned char **_imgData, unsigned int &_width,
    unsigned int &_height)
{
  sensors::SensorPtr sensor = sensors::get_sensor(_cameraName);
  EXPECT_TRUE(sensor != NULL);
  sensors::CameraSensorPtr camSensor =
    boost::dynamic_pointer_cast<sensors::CameraSensor>(sensor);

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
    common::Time::MSleep(100);

  camSensor->GetCamera()->DisconnectNewImageFrame(c);
}

/////////////////////////////////////////////////
physics::ModelPtr ServerFixture::SpawnModel(const msgs::Model &_msg)
{
  physics::WorldPtr world = physics::get_world();
  ServerFixture::CheckPointer(world);
  world->InsertModelString(
    "<sdf version='" + std::string(SDF_VERSION) + "'>"
    + msgs::ModelToSDF(_msg)->ToString("")
    + "</sdf>");

  common::Time wait(10, 0);
  common::Time wallStart = common::Time::GetWallTime();
  unsigned int waitCount = 0;
  while (wait > (common::Time::GetWallTime() - wallStart) &&
         !this->HasEntity(_msg.name()))
  {
    common::Time::MSleep(10);
    if (++waitCount % 100 == 0)
    {
      gzwarn << "Waiting " << waitCount / 100 << " seconds for "
             << "box to spawn." << std::endl;
    }
  }

  return world->GetModel(_msg.name());
}

/////////////////////////////////////////////////
void ServerFixture::SpawnCamera(const std::string &_modelName,
    const std::string &_cameraName,
    const math::Vector3 &_pos, const math::Vector3 &_rpy,
    unsigned int _width, unsigned int _height, double _rate,
    const std::string &_noiseType, double _noiseMean, double _noiseStdDev,
    bool _distortion, double _distortionK1, double _distortionK2,
    double _distortionK3, double _distortionP1, double _distortionP2,
    double _cx, double _cy)
{
  msgs::Factory msg;
  std::ostringstream newModelStr;

  newModelStr << "<sdf version='" << SDF_VERSION << "'>"
    << "<model name ='" << _modelName << "'>"
    << "<static>true</static>"
    << "<pose>" << _pos << " " << _rpy << "</pose>"
    << "<link name ='body'>"
    << "  <sensor name ='" << _cameraName
    << "' type ='camera'>"
    << "    <always_on>1</always_on>"
    << "    <update_rate>" << _rate << "</update_rate>"
    << "    <visualize>true</visualize>"
    << "    <camera>"
    << "      <horizontal_fov>0.78539816339744828</horizontal_fov>"
    << "      <image>"
    << "        <width>" << _width << "</width>"
    << "        <height>" << _height << "</height>"
    << "        <format>R8G8B8</format>"
    << "      </image>"
    << "      <clip>"
    << "        <near>0.1</near><far>100</far>"
    << "      </clip>";
    // << "      <save enabled ='true' path ='/tmp/camera/'/>"

  if (_noiseType.size() > 0)
    newModelStr << "      <noise>"
    << "        <type>" << _noiseType << "</type>"
    << "        <mean>" << _noiseMean << "</mean>"
    << "        <stddev>" << _noiseStdDev << "</stddev>"
    << "      </noise>";

  if (_distortion)
    newModelStr << "      <distortion>"
    << "        <k1>" << _distortionK1 << "</k1>"
    << "        <k2>" << _distortionK2 << "</k2>"
    << "        <k3>" << _distortionK3 << "</k3>"
    << "        <p1>" << _distortionP1 << "</p1>"
    << "        <p2>" << _distortionP2 << "</p2>"
    << "        <center>" << _cx << " " << _cy << "</center>"
    << "      </distortion>";

  newModelStr << "    </camera>"
    << "  </sensor>"
    << "</link>"
    << "</model>"
    << "</sdf>";

  msg.set_sdf(newModelStr.str());
  this->factoryPub->Publish(msg);

  WaitUntilEntitySpawn(_modelName, 100, 50);
  WaitUntilSensorSpawn(_cameraName, 100, 100);
}

/////////////////////////////////////////////////
void ServerFixture::SpawnRaySensor(const std::string &_modelName,
    const std::string &_raySensorName,
    const math::Vector3 &_pos, const math::Vector3 &_rpy,
    double _hMinAngle, double _hMaxAngle,
    double _vMinAngle, double _vMaxAngle,
    double _minRange, double _maxRange,
    double _rangeResolution, unsigned int _samples,
    unsigned int _vSamples, double _hResolution,
    double _vResolution,
    const std::string &_noiseType, double _noiseMean,
    double _noiseStdDev)
{
  msgs::Factory msg;
  std::ostringstream newModelStr;

  newModelStr << "<sdf version='" << SDF_VERSION << "'>"
    << "<model name ='" << _modelName << "'>"
    << "<static>true</static>"
    << "<pose>" << _pos << " " << _rpy << "</pose>"
    << "<link name ='body'>"
    << "<collision name='parent_collision'>"
    << "  <pose>0 0 0.0205 0 0 0</pose>"
    << "  <geometry>"
    << "    <cylinder>"
    << "      <radius>0.021</radius>"
    << "      <length>0.029</length>"
    << "    </cylinder>"
    << "  </geometry>"
    << "</collision>"
    << "  <sensor name ='" << _raySensorName << "' type ='ray'>"
    << "    <ray>"
    << "      <scan>"
    << "        <horizontal>"
    << "          <samples>" << _samples << "</samples>"
    << "          <resolution>" << _hResolution << "</resolution>"
    << "          <min_angle>" << _hMinAngle << "</min_angle>"
    << "          <max_angle>" << _hMaxAngle << "</max_angle>"
    << "        </horizontal>"
    << "        <vertical>"
    << "          <samples>" << _vSamples << "</samples>"
    << "          <resolution>" << _vResolution << "</resolution>"
    << "          <min_angle>" << _vMinAngle << "</min_angle>"
    << "          <max_angle>" << _vMaxAngle << "</max_angle>"
    << "        </vertical>"
    << "      </scan>"
    << "      <range>"
    << "        <min>" << _minRange << "</min>"
    << "        <max>" << _maxRange << "</max>"
    << "        <resolution>" << _rangeResolution <<"</resolution>"
    << "      </range>";

  if (_noiseType.size() > 0)
    newModelStr << "      <noise>"
    << "        <type>" << _noiseType << "</type>"
    << "        <mean>" << _noiseMean << "</mean>"
    << "        <stddev>" << _noiseStdDev << "</stddev>"
    << "      </noise>";

  newModelStr << "    </ray>"
    << "    <visualize>true</visualize>"
    << "  </sensor>"
    << "</link>"
    << "</model>"
    << "</sdf>";

  msg.set_sdf(newModelStr.str());
  this->factoryPub->Publish(msg);

  WaitUntilEntitySpawn(_modelName, 100, 100);
  WaitUntilSensorSpawn(_raySensorName, 100, 100);
}

/////////////////////////////////////////////////
sensors::SonarSensorPtr ServerFixture::SpawnSonar(const std::string &_modelName,
    const std::string &_sonarName,
    const ignition::math::Pose3d &_pose,
    const double _minRange,
    const double _maxRange,
    const double _radius)
{
  msgs::Factory msg;
  std::ostringstream newModelStr;

  newModelStr << "<sdf version='" << SDF_VERSION << "'>"
    << "<model name ='" << _modelName << "'>"
    << "<static>true</static>"
    << "<pose>" << _pose << "</pose>"
    << "<link name ='body'>"
    << "  <sensor name ='" << _sonarName << "' type ='sonar'>"
    << "    <sonar>"
    << "      <min>" << _minRange << "</min>"
    << "      <max>" << _maxRange << "</max>"
    << "      <radius>" << _radius << "</radius>"
    << "    </sonar>"
    << "    <visualize>true</visualize>"
    << "    <always_on>true</always_on>"
    << "  </sensor>"
    << "</link>"
    << "</model>"
    << "</sdf>";

  msg.set_sdf(newModelStr.str());
  this->factoryPub->Publish(msg);

  WaitUntilEntitySpawn(_modelName, 100, 100);
  WaitUntilSensorSpawn(_sonarName, 100, 100);
  return boost::dynamic_pointer_cast<sensors::SonarSensor>(
      sensors::get_sensor(_sonarName));
}


/////////////////////////////////////////////////
void ServerFixture::SpawnGpuRaySensor(const std::string &_modelName,
    const std::string &_raySensorName,
    const math::Vector3 &_pos, const math::Vector3 &_rpy,
    double _hMinAngle, double _hMaxAngle,
    double _minRange, double _maxRange,
    double _rangeResolution, unsigned int _samples,
    const std::string &_noiseType, double _noiseMean,
    double _noiseStdDev)
{
  msgs::Factory msg;
  std::ostringstream newModelStr;

  newModelStr << "<sdf version='" << SDF_VERSION << "'>"
    << "<model name ='" << _modelName << "'>"
    << "<static>true</static>"
    << "<pose>" << _pos << " " << _rpy << "</pose>"
    << "<link name ='body'>"
    << "<collision name='parent_collision'>"
    << "  <pose>0 0 0.0205 0 0 0</pose>"
    << "  <geometry>"
    << "    <cylinder>"
    << "      <radius>0.021</radius>"
    << "      <length>0.029</length>"
    << "    </cylinder>"
    << "  </geometry>"
    << "</collision>"
    << "  <sensor name ='" << _raySensorName
    << "' type ='gpu_ray'>"
    << "    <ray>"
    << "      <scan>"
    << "        <horizontal>"
    << "          <samples>" << _samples << "</samples>"
    << "          <resolution> 1 </resolution>"
    << "          <min_angle>" << _hMinAngle << "</min_angle>"
    << "          <max_angle>" << _hMaxAngle << "</max_angle>"
    << "        </horizontal>"
    << "      </scan>"
    << "      <range>"
    << "        <min>" << _minRange << "</min>"
    << "        <max>" << _maxRange << "</max>"
    << "        <resolution>" << _rangeResolution <<"</resolution>"
    << "      </range>";

  if (_noiseType.size() > 0)
    newModelStr << "      <noise>"
    << "        <type>" << _noiseType << "</type>"
    << "        <mean>" << _noiseMean << "</mean>"
    << "        <stddev>" << _noiseStdDev << "</stddev>"
    << "      </noise>";

  newModelStr << "    </ray>"
    << "  </sensor>"
    << "</link>"
    << "</model>"
    << "</sdf>";

  msg.set_sdf(newModelStr.str());
  this->factoryPub->Publish(msg);

  WaitUntilEntitySpawn(_modelName, 100, 100);
  WaitUntilSensorSpawn(_raySensorName, 100, 100);
}

/////////////////////////////////////////////////
void ServerFixture::SpawnImuSensor(const std::string &_modelName,
    const std::string &_imuSensorName,
    const math::Vector3 &_pos, const math::Vector3 &_rpy,
    const std::string &_noiseType,
    double _rateNoiseMean, double _rateNoiseStdDev,
    double _rateBiasMean, double _rateBiasStdDev,
    double _accelNoiseMean, double _accelNoiseStdDev,
    double _accelBiasMean, double _accelBiasStdDev)
{
  msgs::Factory msg;
  std::ostringstream newModelStr;

  newModelStr << "<sdf version='" << SDF_VERSION << "'>"
    << "<model name ='" << _modelName << "'>" << std::endl
    << "<static>true</static>" << std::endl
    << "<pose>" << _pos << " " << _rpy << "</pose>" << std::endl
    << "<link name ='body'>" << std::endl
    << "<inertial>" << std::endl
    << "<mass>0.1</mass>" << std::endl
    << "</inertial>" << std::endl
    << "<collision name='parent_collision'>" << std::endl
    << "  <pose>0 0 0.0205 0 0 0</pose>" << std::endl
    << "  <geometry>" << std::endl
    << "    <cylinder>" << std::endl
    << "      <radius>0.021</radius>" << std::endl
    << "      <length>0.029</length>" << std::endl
    << "    </cylinder>" << std::endl
    << "  </geometry>" << std::endl
    << "</collision>" << std::endl
    << "  <sensor name ='" << _imuSensorName
    << "' type ='imu'>" << std::endl
    << "    <imu>" << std::endl;

  if (_noiseType.size() > 0)
  {
    newModelStr << "      <noise>" << std::endl
    << "        <type>" << _noiseType << "</type>" << std::endl
    << "        <rate>" << std::endl
    << "          <mean>" << _rateNoiseMean
    << "</mean>" << std::endl
    << "          <stddev>" << _rateNoiseStdDev
    << "</stddev>" << std::endl
    << "          <bias_mean>" << _rateBiasMean
    << "</bias_mean>" << std::endl
    << "          <bias_stddev>" << _rateBiasStdDev
    << "</bias_stddev>" << std::endl
    << "        </rate>" << std::endl
    << "        <accel>" << std::endl
    << "          <mean>" << _accelNoiseMean << "</mean>"
    << std::endl
    << "          <stddev>" << _accelNoiseStdDev << "</stddev>"
    << std::endl
    << "          <bias_mean>" << _accelBiasMean
    << "</bias_mean>" << std::endl
    << "          <bias_stddev>" << _accelBiasStdDev
    << "</bias_stddev>" << std::endl
    << "        </accel>" << std::endl
    << "      </noise>" << std::endl;
  }

  newModelStr << "    </imu>" << std::endl
    << "  </sensor>" << std::endl
    << "</link>" << std::endl
    << "</model>" << std::endl
    << "</sdf>" << std::endl;

  msg.set_sdf(newModelStr.str());
  this->factoryPub->Publish(msg);

  WaitUntilEntitySpawn(_modelName, 100, 1000);
  WaitUntilSensorSpawn(_imuSensorName, 100, 100);
}

/////////////////////////////////////////////////
void ServerFixture::SpawnUnitContactSensor(const std::string &_name,
    const std::string &_sensorName,
    const std::string &_collisionType, const math::Vector3 &_pos,
    const math::Vector3 &_rpy, bool _static)
{
  msgs::Factory msg;
  std::ostringstream newModelStr;
  std::ostringstream shapeStr;

  if (_collisionType == "box")
  {
    shapeStr << " <box><size>1 1 1</size></box>";
  }
  else if (_collisionType == "cylinder")
  {
    shapeStr << "<cylinder>"
             << "  <radius>.5</radius><length>1.0</length>"
             << "</cylinder>";
  }

  newModelStr << "<sdf version='" << SDF_VERSION << "'>"
    << "<model name ='" << _name << "'>"
    << "<static>" << _static << "</static>"
    << "<pose>" << _pos << " " << _rpy << "</pose>"
    << "<link name ='body'>"
    << "  <collision name ='contact_collision'>"
    << "    <geometry>"
    << shapeStr.str()
    << "    </geometry>"
    << "    <surface>"
    << "      <contact>"
    << "        <ode>"
    << "          <min_depth>0.005</min_depth>"
    << "        </ode>"
    << "      </contact>"
    << "    </surface>"
    << "  </collision>"
    << "  <visual name ='visual'>"
    << "    <geometry>"
    << shapeStr.str()
    << "    </geometry>"
    << "  </visual>"
    << "  <sensor name='" << _sensorName << "' type='contact'>"
    << "    <contact>"
    << "      <collision>contact_collision</collision>"
    << "    </contact>"
    << "  </sensor>"
    << "</link>"
    << "</model>"
    << "</sdf>";

  msg.set_sdf(newModelStr.str());
  this->factoryPub->Publish(msg);

  WaitUntilEntitySpawn(_name, 100, 100);
  WaitUntilSensorSpawn(_sensorName, 100, 100);
}

/////////////////////////////////////////////////
void ServerFixture::SpawnUnitImuSensor(const std::string &_name,
    const std::string &_sensorName,
    const std::string &_collisionType,
    const std::string &_topic, const math::Vector3 &_pos,
    const math::Vector3 &_rpy, bool _static)
{
  msgs::Factory msg;
  std::ostringstream newModelStr;
  std::ostringstream shapeStr;
  if (_collisionType == "box")
    shapeStr << " <box><size>1 1 1</size></box>";
  else if (_collisionType == "cylinder")
  {
    shapeStr << "<cylinder>"
             << "  <radius>.5</radius><length>1.0</length>"
             << "</cylinder>";
  }
  newModelStr << "<sdf version='" << SDF_VERSION << "'>"
    << "<model name ='" << _name << "'>"
    << "<static>" << _static << "</static>"
    << "<pose>" << _pos << " " << _rpy << "</pose>"
    << "<link name ='body'>"
    << "  <collision name ='contact_collision'>"
    << "    <geometry>"
    << shapeStr.str()
    << "    </geometry>"
    << "    <surface>"
    << "      <contact>"
    << "        <ode>"
    << "          <min_depth>0.01</min_depth>"
    << "        </ode>"
    << "      </contact>"
    << "    </surface>"
    << "  </collision>"
    << "  <visual name ='visual'>"
    << "    <geometry>"
    << shapeStr.str()
    << "    </geometry>"
    << "  </visual>"
    << "  <sensor name='" << _sensorName << "' type='imu'>"
    << "    <imu>"
    << "      <topic>" << _topic << "</topic>"
    << "    </imu>"
    << "  </sensor>"
    << "</link>"
    << "</model>"
    << "</sdf>";

  msg.set_sdf(newModelStr.str());
  this->factoryPub->Publish(msg);

  WaitUntilEntitySpawn(_name, 20, 50);
  WaitUntilSensorSpawn(_sensorName, 100, 100);
}

/////////////////////////////////////////////////
void ServerFixture::SpawnUnitAltimeterSensor(const std::string &_name,
    const std::string &_sensorName,
    const std::string &_collisionType,
    const std::string &_topic,
    const ignition::math::Vector3d &_pos,
    const ignition::math::Vector3d &_rpy,
    bool _static)
{
  msgs::Factory msg;
  std::ostringstream newModelStr;
  std::ostringstream shapeStr;

  if (_collisionType == "box")
  {
    shapeStr << " <box><size>1 1 1</size></box>";
  }
  else if (_collisionType == "cylinder")
  {
    shapeStr << "<cylinder>"
             << "  <radius>.5</radius><length>1.0</length>"
             << "</cylinder>";
  }

  newModelStr << "<sdf version='" << SDF_VERSION << "'>"
    << "<model name ='" << _name << "'>"
    << "<static>" << _static << "</static>"
    << "<pose>" << _pos << " " << _rpy << "</pose>"
    << "<link name ='body'>"
    << "  <collision name ='contact_collision'>"
    << "    <geometry>"
    << shapeStr.str()
    << "    </geometry>"
    << "  </collision>"
    << "  <visual name ='visual'>"
    << "    <geometry>"
    << shapeStr.str()
    << "    </geometry>"
    << "  </visual>"
    << "  <sensor name='" << _sensorName << "' type='altimeter'>"
    << "    <topic>" << _topic << "</topic>"
    << "    <altimeter>"
    << "    </altimeter>"
    << "  </sensor>"
    << "</link>"
    << "</model>"
    << "</sdf>";

  msg.set_sdf(newModelStr.str());
  this->factoryPub->Publish(msg);

  WaitUntilEntitySpawn(_name, 20, 50);
  WaitUntilSensorSpawn(_sensorName, 100, 100);
}

/////////////////////////////////////////////////
void ServerFixture::SpawnUnitMagnetometerSensor(const std::string &_name,
    const std::string &_sensorName,
    const std::string &_collisionType,
    const std::string &_topic,
    const ignition::math::Vector3d &_pos,
    const ignition::math::Vector3d &_rpy, bool _static)
{
  msgs::Factory msg;
  std::ostringstream newModelStr;
  std::ostringstream shapeStr;
  if (_collisionType == "box")
  {
    shapeStr << " <box><size>1 1 1</size></box>";
  }
  else if (_collisionType == "cylinder")
  {
    shapeStr << "<cylinder>"
             << "  <radius>.5</radius><length>1.0</length>"
             << "</cylinder>";
  }
  newModelStr << "<sdf version='" << SDF_VERSION << "'>"
    << "<model name ='" << _name << "'>"
    << "<static>" << _static << "</static>"
    << "<pose>" << _pos << " " << _rpy << "</pose>"
    << "<link name ='body'>"
    << "  <collision name ='contact_collision'>"
    << "    <geometry>"
    << shapeStr.str()
    << "    </geometry>"
    << "  </collision>"
    << "  <visual name ='visual'>"
    << "    <geometry>"
    << shapeStr.str()
    << "    </geometry>"
    << "  </visual>"
    << "  <sensor name='" << _sensorName << "' type='magnetometer'>"
    << "    <topic>" << _topic << "</topic>"
    << "    <magnetometer>"
    << "    </magnetometer>"
    << "  </sensor>"
    << "</link>"
    << "</model>"
    << "</sdf>";

  msg.set_sdf(newModelStr.str());
  this->factoryPub->Publish(msg);

  WaitUntilEntitySpawn(_name, 20, 50);
  WaitUntilSensorSpawn(_sensorName, 100, 100);
}

/////////////////////////////////////////////////
void ServerFixture::launchTimeoutFailure(const char *_logMsg,
                                         const int _timeoutCS)
{
     FAIL() << "ServerFixture timeout (wait more than " << _timeoutCS / 100
            << "s): " << _logMsg;
}

/////////////////////////////////////////////////
void ServerFixture::SpawnWirelessTransmitterSensor(const std::string &_name,
    const std::string &_sensorName,
    const math::Vector3 &_pos,
    const math::Vector3 &_rpy,
    const std::string &_essid,
    double _freq,
    double _power,
    double _gain,
    bool _visualize)
{
  msgs::Factory msg;
  std::ostringstream newModelStr;

  newModelStr << "<sdf version='" << SDF_VERSION << "'>"
    << "<model name ='" << _name << "'>"
    << "<static>true</static>"
    << "<pose>" << _pos << " " << _rpy << "</pose>"
    << "<link name ='link'>"
    << "  <sensor name='" << _sensorName
    <<         "' type='wireless_transmitter'>"
    << "    <always_on>1</always_on>"
    << "    <update_rate>1</update_rate>"
    << "    <visualize>" << _visualize << "</visualize>"
    << "    <transceiver>"
    << "      <essid>" << _essid << "</essid>"
    << "      <frequency>" << _freq << "</frequency>"
    << "      <power>" << _power << "</power>"
    << "      <gain>" << _gain << "</gain>"
    << "    </transceiver>"
    << "  </sensor>"
    << "</link>"
    << "</model>"
    << "</sdf>";

  msg.set_sdf(newModelStr.str());
  this->factoryPub->Publish(msg);

  WaitUntilEntitySpawn(_name, 100, 100);
  WaitUntilSensorSpawn(_sensorName, 100, 100);
}

/////////////////////////////////////////////////
void ServerFixture::SpawnWirelessReceiverSensor(const std::string &_name,
    const std::string &_sensorName,
    const math::Vector3 &_pos,
    const math::Vector3 &_rpy,
    double _minFreq,
    double _maxFreq,
    double _power,
    double _gain,
    double _sensitivity,
    bool _visualize)
{
  msgs::Factory msg;
  std::ostringstream newModelStr;

  newModelStr << "<sdf version='" << SDF_VERSION << "'>"
    << "<model name ='" << _name << "'>"
    << "<static>true</static>"
    << "<pose>" << _pos << " " << _rpy << "</pose>"
    << "<link name ='link'>"
    << "  <sensor name='" << _sensorName
    <<         "' type='wireless_receiver'>"
    << "    <update_rate>1</update_rate>"
    << "    <visualize>" << _visualize << "</visualize>"
    << "    <transceiver>"
    << "      <min_frequency>" << _minFreq << "</min_frequency>"
    << "      <max_frequency>" << _maxFreq << "</max_frequency>"
    << "      <power>" << _power << "</power>"
    << "      <gain>" << _gain << "</gain>"
    << "      <sensitivity>" << _sensitivity << "</sensitivity>"
    << "    </transceiver>"
    << "  </sensor>"
    << "</link>"
    << "</model>"
    << "</sdf>";

  msg.set_sdf(newModelStr.str());
  this->factoryPub->Publish(msg);

  WaitUntilEntitySpawn(_name, 100, 100);
  WaitUntilSensorSpawn(_sensorName, 100, 100);
}

/////////////////////////////////////////////////
void ServerFixture::WaitUntilEntitySpawn(const std::string &_name,
                        unsigned int _sleepEach,
                        int _retries)
{
  int i = 0;
  // Wait for the entity to spawn
  while (!this->HasEntity(_name) && i < _retries)
  {
    common::Time::MSleep(_sleepEach);
    ++i;
  }
  EXPECT_LT(i, _retries);

  if (i >= _retries)
    FAIL() << "ServerFixture timeout: max number of retries ("
           << _retries
           << ") exceeded while awaiting the spawn of " << _name;
}

/////////////////////////////////////////////////
void ServerFixture::WaitUntilSensorSpawn(const std::string &_name,
                        unsigned int _sleepEach,
                        int _retries)
{
  int i = 0;
  // Wait for the sensor to spawn
  while (!sensors::get_sensor(_name) && i < _retries)
  {
    common::Time::MSleep(_sleepEach);
    ++i;
  }
  EXPECT_LT(i, _retries);

  if (i >= _retries)
    FAIL() << "ServerFixture timeout: max number of retries ("
           << _retries
           << ") exceeded while awaiting the spawn of " << _name;
}

/////////////////////////////////////////////////
void ServerFixture::WaitUntilIteration(const uint32_t _goalIteration,
    const int _sleepEach, const int _retries) const
{
  int i = 0;
  auto world = physics::get_world();
  while ((world->GetIterations() != _goalIteration) && (i < _retries))
  {
    ++i;
    common::Time::MSleep(_sleepEach);
  }
}

/////////////////////////////////////////////////
void ServerFixture::WaitUntilSimTime(const common::Time &_goalTime,
    const int _sleepEach, const int _retries) const
{
  int i = 0;
  auto world = physics::get_world();
  while ((world->GetSimTime() != _goalTime) && (i < _retries))
  {
    ++i;
    common::Time::MSleep(_sleepEach);
  }
}

/////////////////////////////////////////////////
void ServerFixture::SpawnLight(const std::string &_name,
    const std::string &_type,
    const math::Vector3 &_pos, const math::Vector3 &_rpy,
    const common::Color &_diffuse,
    const common::Color &_specular,
    const math::Vector3 &_direction,
    double _attenuationRange,
    double _attenuationConstant,
    double _attenuationLinear,
    double _attenuationQuadratic,
    double _spotInnerAngle,
    double _spotOuterAngle,
    double _spotFallOff,
    bool _castShadows)
{
  msgs::Factory msg;
  std::ostringstream newLightStr;

  newLightStr << "<sdf version='" << SDF_VERSION << "'>"
    << "<light name ='" << _name << "' type = '" << _type << "'>"
    << "<pose>" << _pos << " " << _rpy << "</pose>"
    << "<diffuse>" << _diffuse << "</diffuse>"
    << "<specular>" << _specular << "</specular>"
    << "<direction>" << _direction << "</direction>"
    << "<attenuation>"
    << "  <range>" << _attenuationRange << "</range>"
    << "  <constant>" << _attenuationConstant << "</constant>"
    << "  <linear>" << _attenuationLinear << "</linear>"
    << "  <quadratic>" << _attenuationQuadratic << "</quadratic>"
    << "</attenuation>";

  if (_type == "spot")
  {
    newLightStr << "<spot>"
    << "  <inner_angle>" << _spotInnerAngle << "</inner_angle>"
    << "  <outer_angle>" << _spotOuterAngle << "</outer_angle>"
    << "  <falloff>" << _spotFallOff << "</falloff>"
    << "</spot>";
  }

  newLightStr << "<cast_shadows>" << _castShadows << "</cast_shadows>"
    << "</light>"
    << "</sdf>";

  msg.set_sdf(newLightStr.str());
  this->factoryPub->Publish(msg);

  physics::WorldPtr world = physics::get_world();
  msgs::Scene sceneMsg;
  int timeOutCount = 0;
  int maxTimeOut = 10;
  while (timeOutCount < maxTimeOut)
  {
    sceneMsg = world->GetSceneMsg();
    for (int i = 0; i < sceneMsg.light_size(); ++i)
    {
      if (sceneMsg.light(i).name() == _name)
        break;
    }
    timeOutCount++;
    common::Time::MSleep(100);
  }
}

/////////////////////////////////////////////////
void ServerFixture::SpawnCylinder(const std::string &_name,
    const math::Vector3 &_pos, const math::Vector3 &_rpy,
    bool _static)
{
  msgs::Factory msg;
  std::ostringstream newModelStr;
  msgs::Model model;
  model.set_name(_name);
  model.set_is_static(_static);
  msgs::Set(model.mutable_pose(),
      ignition::math::Pose3d(_pos.Ign(), _rpy.Ign()));
  msgs::AddCylinderLink(model, 1.0, 0.5, 1.0);
  auto link = model.mutable_link(0);
  link->set_name("body");
  link->mutable_collision(0)->set_name("geom");

  newModelStr << "<sdf version='" << SDF_VERSION << "'>"
    << msgs::ModelToSDF(model)->ToString("")
    << "</sdf>";

  msg.set_sdf(newModelStr.str());
  this->factoryPub->Publish(msg);

  // Wait for the entity to spawn
  while (!this->HasEntity(_name))
    common::Time::MSleep(100);
}

/////////////////////////////////////////////////
void ServerFixture::SpawnSphere(const std::string &_name,
    const math::Vector3 &_pos, const math::Vector3 &_rpy,
    bool _wait, bool _static)
{
  msgs::Factory msg;
  std::ostringstream newModelStr;

  newModelStr << "<sdf version='" << SDF_VERSION << "'>"
    << "<model name ='" << _name << "'>"
    << "<static>" << _static << "</static>"
    << "<pose>" << _pos << " " << _rpy << "</pose>"
    << "<link name ='body'>"
    << "  <collision name ='geom'>"
    << "    <geometry>"
    << "      <sphere><radius>.5</radius></sphere>"
    << "    </geometry>"
    << "  </collision>"
    << "  <visual name ='visual'>"
    << "    <geometry>"
    << "      <sphere><radius>.5</radius></sphere>"
    << "    </geometry>"
    << "  </visual>"
    << "</link>"
    << "</model>"
    << "</sdf>";

  msg.set_sdf(newModelStr.str());
  this->factoryPub->Publish(msg);

  // Wait for the entity to spawn
  while (_wait && !this->HasEntity(_name))
    common::Time::MSleep(100);
}

/////////////////////////////////////////////////
void ServerFixture::SpawnSphere(const std::string &_name,
    const math::Vector3 &_pos, const math::Vector3 &_rpy,
    const math::Vector3 &_cog, double _radius,
    bool _wait, bool _static)
{
  msgs::Factory msg;
  std::ostringstream newModelStr;
  msgs::Model model;
  model.set_name(_name);
  model.set_is_static(_static);
  msgs::Set(model.mutable_pose(),
      ignition::math::Pose3d(_pos.Ign(), _rpy.Ign()));
  msgs::AddSphereLink(model, 1.0, _radius);
  auto link = model.mutable_link(0);
  link->set_name("body");
  link->mutable_collision(0)->set_name("geom");
  msgs::Set(link->mutable_inertial()->mutable_pose(),
            ignition::math::Pose3d(_cog.Ign(),
              ignition::math::Quaterniond()));

  newModelStr << "<sdf version='" << SDF_VERSION << "'>"
    << msgs::ModelToSDF(model)->ToString("")
    << "</sdf>";

  msg.set_sdf(newModelStr.str());
  this->factoryPub->Publish(msg);

  // Wait for the entity to spawn
  while (_wait && !this->HasEntity(_name))
    common::Time::MSleep(100);
}

/////////////////////////////////////////////////
void ServerFixture::SpawnBox(const std::string &_name,
    const math::Vector3 &_size, const math::Vector3 &_pos,
    const math::Vector3 &_rpy, bool _static)
{
  msgs::Factory msg;
  std::ostringstream newModelStr;
  msgs::Model model;
  model.set_name(_name);
  model.set_is_static(_static);
  msgs::Set(model.mutable_pose(),
      ignition::math::Pose3d(_pos.Ign(), _rpy.Ign()));
  msgs::AddBoxLink(model, 1.0, _size.Ign());
  auto link = model.mutable_link(0);
  link->set_name("body");
  link->mutable_collision(0)->set_name("geom");

  newModelStr << "<sdf version='" << SDF_VERSION << "'>"
    << msgs::ModelToSDF(model)->ToString("")
    << "</sdf>";

  msg.set_sdf(newModelStr.str());
  this->factoryPub->Publish(msg);

  // Wait for the entity to spawn
  while (!this->HasEntity(_name))
    common::Time::MSleep(100);
}

/////////////////////////////////////////////////
void ServerFixture::SpawnTrimesh(const std::string &_name,
    const std::string &_modelPath, const math::Vector3 &_scale,
    const math::Vector3 &_pos, const math::Vector3 &_rpy,
    bool _static)
{
  msgs::Factory msg;
  std::ostringstream newModelStr;

  newModelStr << "<sdf version='" << SDF_VERSION << "'>"
    << "<model name ='" << _name << "'>"
    << "<static>" << _static << "</static>"
    << "<pose>" << _pos << " " << _rpy << "</pose>"
    << "<link name ='body'>"
    << "  <collision name ='geom'>"
    << "    <geometry>"
    << "      <mesh>"
    << "        <uri>" << _modelPath << "</uri>"
    << "        <scale>" << _scale << "</scale>"
    << "      </mesh>"
    << "    </geometry>"
    << "  </collision>"
    << "  <visual name ='visual'>"
    << "    <geometry>"
    << "      <mesh><uri>" << _modelPath << "</uri></mesh>"
    << "    </geometry>"
    << "  </visual>"
    << "</link>"
    << "</model>"
    << "</sdf>";

  msg.set_sdf(newModelStr.str());
  this->factoryPub->Publish(msg);

  // Wait for the entity to spawn
  while (!this->HasEntity(_name))
    common::Time::MSleep(100);
}

/////////////////////////////////////////////////
void ServerFixture::SpawnEmptyLink(const std::string &_name,
    const math::Vector3 &_pos, const math::Vector3 &_rpy,
    bool _static)
{
  msgs::Factory msg;
  std::ostringstream newModelStr;
  msgs::Model model;
  model.set_name(_name);
  model.set_is_static(_static);
  msgs::Set(model.mutable_pose(),
      ignition::math::Pose3d(_pos.Ign(), _rpy.Ign()));
  model.add_link();
  model.mutable_link(0)->set_name("body");

  newModelStr << "<sdf version='" << SDF_VERSION << "'>"
    << msgs::ModelToSDF(model)->ToString("")
    << "</sdf>";

  msg.set_sdf(newModelStr.str());
  this->factoryPub->Publish(msg);

  // Wait for the entity to spawn
  while (!this->HasEntity(_name))
    common::Time::MSleep(100);
}

/////////////////////////////////////////////////
void ServerFixture::SpawnModel(const std::string &_filename)
{
  msgs::Factory msg;
  msg.set_sdf_filename(_filename);
  this->factoryPub->Publish(msg);
}

/////////////////////////////////////////////////
void ServerFixture::SpawnSDF(const std::string &_sdf)
{
  msgs::Factory msg;
  msg.set_sdf(_sdf);
  this->factoryPub->Publish(msg);

  // The code above sends a message, but it will take some time
  // before the message is processed.
  //
  // The code below parses the sdf string to find a model name,
  // then this function will block until that model
  // has been processed and recognized by the Server Fixture.
  sdf::SDF sdfParsed;
  sdfParsed.SetFromString(_sdf);
  // Check that sdf contains a model
  if (sdfParsed.Root()->HasElement("model"))
  {
    // Timeout of 30 seconds (3000 * 10 ms)
    int waitCount = 0, maxWaitCount = 3000;
    sdf::ElementPtr model = sdfParsed.Root()->GetElement("model");
    std::string name = model->Get<std::string>("name");
    while (!this->HasEntity(name) && ++waitCount < maxWaitCount)
      common::Time::MSleep(100);
    ASSERT_LT(waitCount, maxWaitCount);
  }
}

/////////////////////////////////////////////////
void ServerFixture::LoadPlugin(const std::string &_filename,
                const std::string &_name)
{
  // Get the first world...we assume it the only one running
  physics::WorldPtr world = physics::get_world();
  world->LoadPlugin(_filename, _name, sdf::ElementPtr());
}

/////////////////////////////////////////////////
physics::ModelPtr ServerFixture::GetModel(const std::string &_name)
{
  // Get the first world...we assume it the only one running
  physics::WorldPtr world = physics::get_world();
  return world->GetModel(_name);
}

/////////////////////////////////////////////////
void ServerFixture::RemoveModel(const std::string &_name)
{
  msgs::Request *msg = msgs::CreateRequest("entity_delete", _name);
  this->requestPub->Publish(*msg);
  delete msg;
}

/////////////////////////////////////////////////
void ServerFixture::RemovePlugin(const std::string &_name)
{
  // Get the first world...we assume it the only one running
  physics::WorldPtr world = physics::get_world();
  world->RemovePlugin(_name);
}

/////////////////////////////////////////////////
void ServerFixture::GetMemInfo(double &_resident, double &_share)
{
#ifdef __linux__
  int totalSize, residentPages, sharePages;
  totalSize = residentPages = sharePages = 0;

  std::ifstream buffer("/proc/self/statm");
  buffer >> totalSize >> residentPages >> sharePages;
  buffer.close();

  // in case x86-64 is configured to use 2MB pages
  int64_t pageSizeKb = sysconf(_SC_PAGE_SIZE) / 1024;

  _resident = residentPages * pageSizeKb;
  _share = sharePages * pageSizeKb;
#elif __MACH__
  // /proc is only available on Linux
  // for OSX, use task_info to get resident and virtual memory
  struct task_basic_info t_info;
  mach_msg_type_number_t t_info_count = TASK_BASIC_INFO_COUNT;
  if (KERN_SUCCESS != task_info(mach_task_self(),
                                TASK_BASIC_INFO,
                                (task_info_t)&t_info,
                                &t_info_count))
  {
    gzerr << "failure calling task_info\n";
    return;
  }
  _resident = static_cast<double>(t_info.resident_size/1024);
  _share = static_cast<double>(t_info.virtual_size/1024);
#else
  gzerr << "Unsupported architecture\n";
  return;
#endif
}

/////////////////////////////////////////////////
std::string ServerFixture::GetUniqueString(const std::string &_prefix)
{
  std::ostringstream stream;
  stream << _prefix << this->uniqueCounter++;
  return stream.str();
}
