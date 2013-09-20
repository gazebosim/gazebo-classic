/*
 * Copyright 2012 Open Source Robotics Foundation
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
#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/sensors/SensorsIface.hh"
#include "ModelPropShop.hh"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_SYSTEM_PLUGIN(ModelPropShop)

/////////////////////////////////////////////
/// \brief Destructor
ModelPropShop::~ModelPropShop()
{
  rendering::fini();
}

/////////////////////////////////////////////
void ModelPropShop::Load(int /*_argc*/, char ** /*_argv*/)
{
}

/////////////////////////////////////////////
void ModelPropShop::Init()
{
  this->connections.push_back(
      event::Events::ConnectWorldUpdateBegin(
        boost::bind(&ModelPropShop::Update, this)));

  // Turn off sensors.
  gazebo::sensors::stop();
  gazebo::sensors::fini();

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
  this->pub = this->node->Advertise<msgs::ServerControl>(
      "/gazebo/server/control");
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
    this->camera->SetHFOV(GZ_DTOR(60));
    this->camera->SetImageWidth(640);
    this->camera->SetImageHeight(480);
    this->camera->CreateRenderTexture("ModelPropShop_RttTex");
  }

  event::Events::preRender();

  if (this->scene->GetInitialized())
  {
    rendering::VisualPtr vis = this->scene->GetVisual("pr2");
    if (vis)
    {
      math::Box bbox = vis->GetBoundingBox();

      // Compute model scaling.
      double scaling = 1.0/ bbox.GetSize().GetMax();

      // Compute the model translation.
      math::Vector3 trans = bbox.GetCenter();
      trans *= -scaling;

      // Normalize the size of the visual
      vis->SetScale(math::Vector3(scaling, scaling, scaling));
      vis->SetWorldPose(math::Pose(trans.x, trans.y, trans.z, 0, 0, 0));

      // Place the visual at the origin
      bbox = vis->GetBoundingBox();

      math::Pose pose;

      // Top view
      pose.pos.Set(0, 0, 1.8);
      pose.rot.SetFromEuler(0, GZ_DTOR(90), 0);
      this->camera->SetWorldPose(pose);
      this->camera->Update();
      this->camera->Render(true);
      this->camera->PostRender();
      this->camera->SaveFrame("/tmp/top_view.png");

      // Front view
      pose.pos.Set(1.8, 0, 0);
      pose.rot.SetFromEuler(0, 0, GZ_DTOR(-180));
      this->camera->SetWorldPose(pose);
      this->camera->Update();
      this->camera->Render(true);
      this->camera->PostRender();
      this->camera->SaveFrame("/tmp/front_view.png");

      // Side view
      pose.pos.Set(0, 1.8, 0);
      pose.rot.SetFromEuler(0, 0, GZ_DTOR(-90));
      this->camera->SetWorldPose(pose);
      this->camera->Update();
      this->camera->Render(true);
      this->camera->PostRender();
      this->camera->SaveFrame("/tmp/side_view.png");

      // Back view
      pose.pos.Set(-1.8, 0, 0);
      pose.rot.SetFromEuler(0, 0, 0);
      this->camera->SetWorldPose(pose);
      this->camera->Update();
      this->camera->Render(true);
      this->camera->PostRender();
      this->camera->SaveFrame("/tmp/back_view.png");

      // Perspective view
      pose.pos.Set(0.9, -0.9, 0.9);
      pose.rot.SetFromEuler(0, GZ_DTOR(30), GZ_DTOR(-220));
      this->camera->SetWorldPose(pose);
      this->camera->Update();
      this->camera->Render(true);
      this->camera->PostRender();
      this->camera->SaveFrame("/tmp/perspective_view.png");

      // Clean up the camera.
      this->camera.reset();
      this->scene->RemoveCamera("propshopcamera");

      // Tell the server to stop.
      msgs::ServerControl msg;
      msg.set_stop(true);
      this->pub->Publish(msg);
    }
  }
}
