/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
#include <boost/algorithm/string.hpp>
#include "rendering/Visual.hh"
#include "rendering/Heightmap.hh"
#include "rendering/RTShaderSystem.hh"
#include "rendering/Rendering.hh"
#include "rendering/UserCamera.hh"
#include "rendering/Light.hh"
#include "rendering/Scene.hh"
#include "rendering/MovableText.hh"
#include "rendering/Light.hh"
#include "transport/transport.h"

#include "movie.hh"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_SYSTEM_PLUGIN(Movie)

/////////////////////////////////////////////////
Movie::Movie()
{
  this->debug = false;
  this->joined = false;
  this->stop = false;
}

/////////////////////////////////////////////////
Movie::~Movie()
{
}

/////////////////////////////////////////////////
void Movie::Load()
{
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
  this->jointAnimPub =
    this->node->Advertise<msgs::JointAnimation>("~/joint_animation");
  this->poseAnimPub =
    this->node->Advertise<msgs::PoseAnimation>("~/pose_animation");

  this->modelPub = this->node->Advertise<msgs::Model>("~/model/modify");

  this->connections.push_back( 
      event::Events::ConnectPreRender( 
        boost::bind(&Movie::PreRender, this) ) );

  this->scene = rendering::get_scene("default");
}


/////////////////////////////////////////////////
void Movie::Init()
{
  this->userCamera = gui::get_active_camera();

  if (this->debug)
  { 
    this->userCamera->SetWorldPose(
        math::Pose(161.93, -99.14, 3.58, 0, GZ_DTOR(1.94), GZ_DTOR(95.12)));
  }
  else
  {
    this->camera = this->scene->CreateCamera("recorder");
    this->camera->Load();
    this->camera->Init();
    this->camera->SetClipDist(1, 100);
    this->camera->SetHFOV(1.047);
    this->camera->SetImageSize(1920, 1080);
    this->camera->CreateRenderTexture("RTT");
    this->camera->EnableSaveFrame(true);
    this->camera->SetSaveFramePathname("/tmp/drc/");

    this->camera->SetWorldPose(math::Pose(-180, -202, 20, 0, 0, 1.75));

    this->userCamera->SetWorldPose(math::Pose(-180, -202, 20, 0, 0, 1.75));
  }
}

/////////////////////////////////////////////////
void Movie::OnCamComplete()
{
  /*msgs::JointAnimation msg;
  msg.set_model_name("pr2");

  this->jointAnimPub->Publish(msg);
  */
}

void Movie::OnCamPioneerComplete()
{

  /*msgs::PoseAnimation msg;
  msg.set_model_name("box1");
  this->poseAnimPub->Publish(msg);
  */

  /*this->viewPoses.clear();
  this->viewPoses.push_back(this->userCamera->GetWorldPose());

  this->viewPoses.push_back(
      math::Pose(180.38, -75.81, 4.02, 0, GZ_DTOR(25.20), GZ_DTOR(139.47)));

  this->userCamera->MoveToPositions(this->viewPoses, 15.0,
        boost::bind(&Movie::OnCamPioneerComplete, this));
        */
}

/////////////////////////////////////////////////
void Movie::PreRender()
{
  if (this->startTime == common::Time(0,0) && this->scene->GetHeightmap())
  {
    if (!this->debug)
    {
      std::vector<math::Pose> quadPoses;

      rendering::Heightmap *map = this->scene->GetHeightmap();

      double imageX[37] = {76, 72, 67, 63, 62, 69, 76, 82, 91, 100, 108, 118, 127, 134,146, 158, 172, 186, 198, 206, 209, 212, 221, 234, 245, 259, 275, 295, 318, 322, 323, 323, 329, 346, 364, 391, 412};
      double imageY[37] = {458, 436, 411, 374, 353, 340, 328, 318, 313, 312, 319, 331, 338, 343, 345, 348, 349, 347, 341, 331, 320, 311, 304, 291, 286, 282, 279, 281, 289, 298, 319, 334, 343, 351, 354, 353, 349};

      for (unsigned int i=0; i < 36; i++)
      {
        double wx = imageX[i] - 256;
        double wy = 256-imageY[i];

        double wx1 = imageX[i+1] - 256;
        double wy1 = 256-imageY[i+1];

        double theta = atan2(wy1 - wy, wx1 - wx);
        double z = map->GetHeight(wx, wy);


        if (z < 20)
          this->viewPoses.push_back(
              math::Pose(wx, wy, 25, 0, GZ_DTOR(10), theta));
        else if (z < 30)
          this->viewPoses.push_back(
              math::Pose(wx, wy, 35, 0, GZ_DTOR(10), theta));
        else
          this->viewPoses.push_back(
              math::Pose(wx, wy, z+4, 0, GZ_DTOR(10), theta));

        quadPoses.push_back(
            math::Pose(wx+.5, wy+.5, z+4, 0, 0, theta));

      }

      this->viewPoses.push_back(
          math::Pose(161.93, -102.14, 3.58, 0, GZ_DTOR(3), GZ_DTOR(95.12)));

      quadPoses.push_back(
          math::Pose(159.89, -94.83, 10.18, 0, 0, GZ_DTOR(90)));

      rendering::VisualPtr quadRotor = this->scene->GetVisual("quadrotor");
      quadRotor->MoveToPositions(quadPoses, 35.0,
          boost::bind(&Movie::OnRotorComplete, this));
    }

    this->startTime = common::Time::GetWallTime();
  }

  if (!this->debug)
  {
    if (common::Time::GetWallTime() - this->startTime > 0.8 &&
        this->viewPoses.size() > 0)
    {
      this->userCamera->MoveToPositions(this->viewPoses, 35.0,
          boost::bind(&Movie::OnCamComplete, this));
      this->camera->MoveToPositions(this->viewPoses, 35.0);


      this->viewPoses.clear();
    }
  }

  //this->scene->PrintSceneGraph();
  //usleep(1000000);
  rendering::VisualPtr fingerVis = this->scene->GetVisual("pr2::r_gripper_r_finger_link");
  rendering::VisualPtr boxVis = this->scene->GetVisual("box1");

  if (fingerVis && boxVis && !this->stop)
  {
    if (!this->joined)
    {
      if (fingerVis->GetWorldPose().pos.Distance(
            boxVis->GetWorldPose().pos) < 0.1)
      {
        this->joined = true;
        this->posOffset = boxVis->GetWorldPose().pos -
                          fingerVis->GetWorldPose().pos;
        this->posOffset.x -= 0.025;
        std::cout << "PoseOffset[" << this->posOffset << "]\n";
      }
    }
    else
    {
      std::cout << "Set[" <<
        fingerVis->GetWorldPose().CoordPositionAdd(this->posOffset) << "]\n";
      boxVis->SetPosition(fingerVis->GetWorldPose().CoordPositionAdd(
            this->posOffset));
      boxVis->SetRotation(fingerVis->GetWorldPose().rot);
      if (fingerVis->GetWorldPose().CoordPositionAdd(this->posOffset).z <= 2.26)
      {
        this->stop = true;
      }
    }
  }

  common::Time currTime = common::Time::GetWallTime();
}

void Movie::OnRotorComplete()
{
  std::vector<math::Pose> quadPoses;

  quadPoses.push_back(
      math::Pose(159.89, -94.83, 5.18, 0, 0, GZ_DTOR(0)));
  quadPoses.push_back(
      math::Pose(159.89, -94.83, 3.18, 0, 0, GZ_DTOR(45)));
  quadPoses.push_back(
      math::Pose(159.89, -94.83, 2.18, 0, 0, GZ_DTOR(90)));

  rendering::VisualPtr quadRotor = this->scene->GetVisual("quadrotor");
  quadRotor->MoveToPositions(quadPoses, 5.0,
      boost::bind(&Movie::OnRotorLand, this));
}

void Movie::OnRotorLand()
{
  std::vector<math::Pose> poses;

  msgs::PoseAnimation msg;
  msg.set_model_name("pioneer2dx");

  double step = 25.0 / 7.0;

  msgs::Set(msg.add_time(), common::Time(step * msg.pose_size(), 0));
  msgs::Set(msg.add_pose(), math::Pose(161, -92, 2, 0, 0, GZ_DTOR(90)));

  msgs::Set(msg.add_time(), common::Time(step * msg.pose_size(), 0));
  msgs::Set(msg.add_pose(), math::Pose(161, -79, 2, 0, 0, GZ_DTOR(90)));

  msgs::Set(msg.add_time(), common::Time(step * msg.pose_size(), 0));
  msgs::Set(msg.add_pose(), math::Pose(162, -77.7, 2, 0, 0, GZ_DTOR(0)));

  msgs::Set(msg.add_time(), common::Time(step * msg.pose_size(), 0));
  msgs::Set(msg.add_pose(), math::Pose(171, -77.7, 2, 0, 0, GZ_DTOR(0)));

  msgs::Set(msg.add_time(), common::Time(step * msg.pose_size(), 0));
  msgs::Set(msg.add_pose(), math::Pose(183, -80.0, 2, 0, 0, GZ_DTOR(0)));

  msgs::Set(msg.add_time(), common::Time(step * msg.pose_size(), 0));
  msgs::Set(msg.add_pose(), math::Pose(186, -80.0, 2, 0, 0, GZ_DTOR(0)));

  this->poseAnimPub->Publish(msg);


  this->viewPoses.clear();
  this->viewPoses.push_back(this->userCamera->GetWorldPose());

  this->viewPoses.push_back(
      math::Pose(160.41, -90.14, 6.61, 0, GZ_DTOR(23.80), GZ_DTOR(79.70)));

  this->viewPoses.push_back(
      math::Pose(157.67, -77.00, 8.56, 0, GZ_DTOR(41.90), GZ_DTOR(-17.70)));

  this->viewPoses.push_back(
      math::Pose(181.24, -81.87, 5.07, 0, GZ_DTOR(41.70), GZ_DTOR(126.50)));

  this->viewPoses.push_back(
      math::Pose(181.38, -76.81, 4.02, 0, GZ_DTOR(26.20), GZ_DTOR(139.47)));

  msgs::PoseAnimation msg2;
  msg2.set_model_name("pr2");
  this->poseAnimPub->Publish(msg2);

  this->userCamera->MoveToPositions(this->viewPoses, 17.0,
        boost::bind(&Movie::OnCamPioneerComplete, this));
  this->camera->MoveToPositions(this->viewPoses, 17.0);
}
