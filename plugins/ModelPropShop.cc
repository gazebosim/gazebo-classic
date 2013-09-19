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
#include "gazebo/sensors/SensorsIface.hh"
#include "ModelPropShop.hh"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_SYSTEM_PLUGIN(ModelPropShop)

/////////////////////////////////////////////
void ModelPropShop::Load(int /*_argc*/, char ** /*_argv*/)
{
  printf("ModelPropShop::Load\n");
}

/////////////////////////////////////////////
void ModelPropShop::Init()
{
  this->connections.push_back(
      event::Events::ConnectWorldUpdateBegin(
        boost::bind(&ModelPropShop::Update, this)));

  this->renderCount = 0;
  gazebo::sensors::stop();
  gazebo::sensors::fini();
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
    this->camera = this->scene->CreateCamera("propshopcamera", true);
    this->camera->SetCaptureData(true);
    this->camera->Load(cameraSDF);
    this->camera->Init();
    this->camera->CreateRenderTexture("ModelPropShop_RttTex");
  }

  event::Events::preRender();

  if (this->scene->GetInitialized())
  {
    /*if (this->renderCount < 100)
    {
      this->renderCount++;
      return;
    }*/

    rendering::VisualPtr vis = this->scene->GetVisual("pr2");
    if (vis)
    {
      math::Box bbox = vis->GetBoundingBox();
      std::cout << "Box[" << bbox << "]\n";

      // Normalize the size of the visual
      vis->SetScale(math::Vector3(1, 1, 1) / bbox.GetSize());

      // Place the visual at the origin
      vis->SetWorldPose(math::Pose(0, 0, 0, 0, 0, 0));

      math::Pose pose;

      // Top view
      pose.pos.Set(0, 0, 2.0);
      pose.rot.SetFromEuler(0, GZ_DTOR(90), 0);
      this->camera->SetWorldPose(pose);
      this->camera->Update();
      this->camera->Render();
      this->camera->PostRender();
      this->camera->SaveFrame("/tmp/top_view.png");

      // Front view
      pose.pos.Set(1.5, 0, 0.5);
      pose.rot.SetFromEuler(0, 0, GZ_DTOR(-180));
      this->camera->Update();
      this->camera->Render();
      this->camera->PostRender();
      this->camera->SaveFrame("/tmp/front_view.png");

      // Side view
      pose.pos.Set(0, 1.5, 0.5);
      pose.rot.SetFromEuler(0, 0, GZ_DTOR(-90));
      this->camera->SetWorldPose(pose);
      this->camera->Update();
      this->camera->Render();
      this->camera->PostRender();
      this->camera->SaveFrame("/tmp/side_view.png");

      // Back view
      pose.pos.Set(-1.5, 0, 0.5);
      pose.rot.SetFromEuler(0, 0, 0);
      this->camera->SetWorldPose(pose);
      this->camera->Update();
      this->camera->Render();
      this->camera->PostRender();
      this->camera->SaveFrame("/tmp/back_view.png");

      rendering::fini();
      gazebo::stop();
      gazebo::fini();
    }
  }
}
