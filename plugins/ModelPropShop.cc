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

#include <boost/program_options.hpp>

#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/sensors/SensorsIface.hh"
#include "ModelPropShop.hh"

using namespace gazebo;
namespace po = boost::program_options;

// Register this plugin with the simulator
GZ_REGISTER_SYSTEM_PLUGIN(ModelPropShop)

/////////////////////////////////////////////
ModelPropShop::~ModelPropShop()
{
  rendering::fini();
}

/////////////////////////////////////////////
void ModelPropShop::Load(int _argc, char **_argv)
{
  // Turn off sensors.
  gazebo::sensors::disable();

  po::options_description v_desc("Options");
  v_desc.add_options()
    ("propshop-save", po::value<std::string>(),
     "Path to save image files into.")
    ("propshop-model", po::value<std::string>(), "Model to spawn.");

  po::options_description desc("Options");
  desc.add(v_desc);

  po::variables_map vm;
  try
  {
    po::store(po::command_line_parser(_argc, _argv).options(
          desc).allow_unregistered().run(), vm);
    po::notify(vm);
  } catch(boost::exception &_e)
  {
    std::cerr << "Error. Invalid arguments\n";
    return;
  }

  // Get the directory in which to save the images.
  if (vm.count("propshop-save"))
  {
    this->savePath = boost::filesystem::path(
        vm["propshop-save"].as<std::string>());
    if (!boost::filesystem::exists(this->savePath))
      boost::filesystem::create_directories(this->savePath);
  }
  else
    this->savePath = boost::filesystem::temp_directory_path();

  std::string modelFile;

  if (vm.count("propshop-model"))
    modelFile = vm["propshop-model"].as<std::string>();
  else
    return;

  std::ifstream ifs(modelFile.c_str());
  if (!ifs)
  {
    std::cerr << "Error: Unable to open file[" << modelFile << "]\n";
    return;
  }

  this->sdf.reset(new sdf::SDF());
  if (!sdf::init(this->sdf))
  {
    std::cerr << "ERROR: SDF parsing the xml failed" << std::endl;
    return;
  }

  if (!sdf::readFile(modelFile, this->sdf))
  {
    std::cerr << "Error: SDF parsing the xml failed\n";
    return;
  }

  sdf::ElementPtr modelElem = this->sdf->Root()->GetElement("model");
  this->modelName = modelElem->Get<std::string>("name");
}

/////////////////////////////////////////////
void ModelPropShop::Init()
{
  this->worldCreatedConn = event::Events::ConnectWorldCreated(
        boost::bind(&ModelPropShop::OnWorldCreated, this));

  this->updateConn = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&ModelPropShop::Update, this));

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
  this->pub = this->node->Advertise<msgs::ServerControl>(
      "/gazebo/server/control");

  this->factoryPub = this->node->Advertise<msgs::Factory>("~/factory");
}

/////////////////////////////////////////////
void ModelPropShop::OnWorldCreated()
{
  this->factoryPub->WaitForConnection();

  if (this->sdf)
  {
    msgs::Factory msg;
    msg.set_sdf(this->sdf->ToString());
    this->factoryPub->Publish(msg, true);
  }
}

/////////////////////////////////////////////
void ModelPropShop::Update()
{
  // Make sure to initialize the rendering engine in the same thread that will
  // capture images.
  if (!this->scene)
  {
    rendering::load();
    rendering::init();

    sdf::ElementPtr cameraSDF(new sdf::Element);
    sdf::initFile("camera.sdf", cameraSDF);

    this->scene = rendering::create_scene("default", false, true);
    this->camera = this->scene->CreateCamera("propshopcamera", false);
    this->camera->SetCaptureData(true);
    this->camera->Load(cameraSDF);
    this->camera->Init();
    this->camera->SetHFOV(static_cast<ignition::math::Angle>(IGN_DTOR(60)));
    this->camera->SetImageWidth(960);
    this->camera->SetImageHeight(540);
    this->camera->CreateRenderTexture("ModelPropShop_RttTex");

    // Create a light
    gazebo::msgs::Light lightMsg;
    lightMsg.set_name("propshop_light");
    lightMsg.set_type(gazebo::msgs::Light::DIRECTIONAL);
    gazebo::msgs::Set(lightMsg.mutable_diffuse(),
                      gazebo::common::Color(1, 1, 1, 1));
    gazebo::msgs::Set(lightMsg.mutable_specular(),
                      gazebo::common::Color(.2, .2, .2, 1));
    gazebo::msgs::Set(lightMsg.mutable_direction(),
                      ignition::math::Vector3d(-0.5, 0.1, -0.9));
    lightMsg.set_cast_shadows(false);
    lightMsg.set_range(1000);
    lightMsg.set_attenuation_constant(1);
    lightMsg.set_attenuation_linear(0);
    lightMsg.set_attenuation_quadratic(0);

    this->light.reset(new gazebo::rendering::Light(this->scene));
    light->LoadFromMsg(lightMsg);
    gazebo::rendering::RTShaderSystem::Instance()->UpdateShaders();
    return;
  }

  if (this->camera && this->scene)
    event::Events::preRender();

  if (this->camera && this->scene->GetInitialized() &&
      this->camera->GetInitialized())
  {
    rendering::VisualPtr vis = this->scene->GetVisual(this->modelName);
    if (vis)
    {
      math::Box bbox = vis->GetBoundingBox();

      // Compute model scaling.
      double scaling = 1.0 / bbox.GetSize().GetMax();

      // Compute the model translation.
      math::Vector3 trans = bbox.GetCenter();
      trans *= -scaling;

      // Normalize the size of the visual
      vis->SetScale(math::Vector3(scaling, scaling, scaling));
      vis->SetWorldPose(math::Pose(trans.x, trans.y, trans.z, 0, 0, 0));

      // Place the visual at the origin
      bbox = vis->GetBoundingBox();

      ignition::math::Pose3d pose;

      // Perspective view
      pose.Pos().Set(1.6, -1.6, 1.2);
      pose.Rot().Euler(0, IGN_DTOR(30), IGN_DTOR(-225));
      this->light->SetDirection(math::Vector3(-0.4, 0.4, -0.4));
      this->camera->SetWorldPose(pose);
      this->camera->Update();
      this->camera->Render(true);
      this->camera->PostRender();
      this->camera->SaveFrame((this->savePath / "1.png").string());

      // Top view
      pose.Pos().Set(0, 0, 2.2);
      pose.Rot().Euler(0, IGN_DTOR(90), 0);
      this->light->SetDirection(math::Vector3(0, 0, -1.0));
      this->camera->SetWorldPose(pose);
      this->camera->Update();
      this->camera->Render(true);
      this->camera->PostRender();
      this->camera->SaveFrame((this->savePath / "2.png").string());

      // Front view
      pose.Pos().Set(2.2, 0, 0);
      pose.Rot().Euler(0, 0, IGN_DTOR(-180));
      this->light->SetDirection(math::Vector3(-0.6, 0.0, -0.4));
      this->camera->SetWorldPose(pose);
      this->camera->Update();
      this->camera->Render(true);
      this->camera->PostRender();
      this->camera->SaveFrame((this->savePath / "3.png").string());

      // Side view
      pose.Pos().Set(0, 2.2, 0);
      pose.Rot().Euler(0, 0, IGN_DTOR(-90));
      this->light->SetDirection(math::Vector3(0, -0.6, -0.4));
      this->camera->SetWorldPose(pose);
      this->camera->Update();
      this->camera->Render(true);
      this->camera->PostRender();
      this->camera->SaveFrame((this->savePath / "4.png").string());

      // Back view
      pose.Pos().Set(-2.2, 0, 0);
      pose.Rot().Euler(0, 0, 0);
      this->light->SetDirection(math::Vector3(0.6, 0, -0.4));
      this->camera->SetWorldPose(pose);
      this->camera->Update();
      this->camera->Render(true);
      this->camera->PostRender();
      this->camera->SaveFrame((this->savePath / "5.png").string());

      event::Events::DisconnectWorldCreated(this->worldCreatedConn);
      this->worldCreatedConn.reset();

      event::Events::DisconnectWorldUpdateBegin(this->updateConn);
      this->updateConn.reset();

      // Clean up the camera.
      this->camera.reset();
      this->light.reset();
      this->scene->RemoveCamera("propshopcamera");

      // Tell the server to stop.
      msgs::ServerControl msg;
      msg.set_stop(true);
      this->pub->Publish(msg);
    }
  }
}
