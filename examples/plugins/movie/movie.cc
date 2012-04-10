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

  this->connections.push_back( 
      event::Events::ConnectPreRender( 
        boost::bind(&Movie::PreRender, this) ) );

  this->scene = rendering::get_scene("default");

  /*
  this->spot1Pattern.push_back(
      std::make_pair(common::Time(1,0), common::Color(.2, .2, .2)));
  this->spot1Pattern.push_back(
      std::make_pair(common::Time(1,100000000), common::Color(0, 0, 0)));
  this->spot1Pattern.push_back(
      std::make_pair(common::Time(1,400000000), common::Color(.2, .2, .2)));
  this->spot1Pattern.push_back(
      std::make_pair(common::Time(1,800000000), common::Color(0, 0, 0)));
  this->spot1Pattern.push_back(
      std::make_pair(common::Time(2,200000000), common::Color(.5, .5, .5)));
  this->spot1Pattern.push_back(
      std::make_pair(common::Time(2,500000000), common::Color(0, 0, 0)));
  this->spot1Pattern.push_back(
      std::make_pair(common::Time(3,0), common::Color(.8, .8, .8)));

  this->spot2Pattern.push_back(
      std::make_pair(common::Time(0,800000000), common::Color(.2, .2, .2)));
  this->spot2Pattern.push_back(
      std::make_pair(common::Time(0,900000000), common::Color(0, 0, 0)));
  this->spot2Pattern.push_back(
      std::make_pair(common::Time(1,200000000), common::Color(.2, .2, .2)));
  this->spot2Pattern.push_back(
      std::make_pair(common::Time(1,800000000), common::Color(0, 0, 0)));
  this->spot2Pattern.push_back(
      std::make_pair(common::Time(1,900000000), common::Color(.5, .5, .5)));
  this->spot2Pattern.push_back(
      std::make_pair(common::Time(2,200000000), common::Color(0, 0, 0)));
  this->spot2Pattern.push_back(
      std::make_pair(common::Time(2,500000000), common::Color(.8, .8, .8)));

  this->spot3Pattern.push_back(
      std::make_pair(common::Time(0,900000000), common::Color(.2, .2, .2)));
  this->spot3Pattern.push_back(
      std::make_pair(common::Time(1,200000000), common::Color(0, 0, 0)));
  this->spot3Pattern.push_back(
      std::make_pair(common::Time(1,600000000), common::Color(.2, .2, .2)));
  this->spot3Pattern.push_back(
      std::make_pair(common::Time(1,900000000), common::Color(0, 0, 0)));
  this->spot3Pattern.push_back(
      std::make_pair(common::Time(2,0), common::Color(.5, .5, .5)));
  this->spot3Pattern.push_back(
      std::make_pair(common::Time(2,100000000), common::Color(0, 0, 0)));
  this->spot3Pattern.push_back(
      std::make_pair(common::Time(2,300000000), common::Color(.8, .8, .8)));

  this->spot4Pattern.push_back(
      std::make_pair(common::Time(0,200000000), common::Color(.2, .2, .2)));
  this->spot4Pattern.push_back(
      std::make_pair(common::Time(0,500000000), common::Color(0, 0, 0)));
  this->spot4Pattern.push_back(
      std::make_pair(common::Time(0,800000000), common::Color(.2, .2, .2)));
  this->spot4Pattern.push_back(
      std::make_pair(common::Time(1,100000000), common::Color(0, 0, 0)));
  this->spot4Pattern.push_back(
      std::make_pair(common::Time(1,5), common::Color(.5, .5, .5)));
  this->spot4Pattern.push_back(
      std::make_pair(common::Time(1,600000000), common::Color(0, 0, 0)));
  this->spot4Pattern.push_back(
      std::make_pair(common::Time(2,100000000), common::Color(.8, .8, .8)));
  
  this->spot5Pattern.push_back(
      std::make_pair(common::Time(0,400000000), common::Color(.2, .2, .2)));
  this->spot5Pattern.push_back(
      std::make_pair(common::Time(0,600000000), common::Color(0, 0, 0)));
  this->spot5Pattern.push_back(
      std::make_pair(common::Time(0,900000000), common::Color(.2, .2, .2)));
  this->spot5Pattern.push_back(
      std::make_pair(common::Time(1,0), common::Color(0, 0, 0)));
  this->spot5Pattern.push_back(
      std::make_pair(common::Time(1,1), common::Color(.5, .5, .5)));
  this->spot5Pattern.push_back(
      std::make_pair(common::Time(1,400000000), common::Color(0, 0, 0)));
  this->spot5Pattern.push_back(
      std::make_pair(common::Time(1,900000000), common::Color(.8, .8, .8)));

  this->viewPoses.push_back(math::Pose(-3.49, 5.48, 7.02,
        0, GZ_DTOR(47.64), GZ_DTOR(-38.62)));
  this->viewPoses.push_back(math::Pose(-5, 0, 1,
        0, GZ_DTOR(11.31), 0));
  */

}


/////////////////////////////////////////////////
void Movie::Init()
{
  this->userCamera = gui::get_active_camera();
  //this->userCamera->SetWorldPose(math::Pose(-180, -202, 20, 0, 0, 1.75));

  this->userCamera->SetWorldPose(
      math::Pose(156.84, -97.43, 4.56, 0, GZ_DTOR(25.45), GZ_DTOR(59.02)));
}

/////////////////////////////////////////////////
void Movie::OnCamComplete()
{
  /*msgs::JointAnimation msg;
  msg.set_model_name("pr2");

  this->jointAnimPub->Publish(msg);
  */
}

/////////////////////////////////////////////////
void Movie::PreRender()
{

  if (this->startTime == common::Time(0,0) && this->scene->GetHeightmap())
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
        math::Pose(156.84, -97.43, 4.56, 0, GZ_DTOR(25.45), GZ_DTOR(59.02)));

    quadPoses.push_back(
        math::Pose(159.89, -94.83, 10.18, 0, 0, GZ_DTOR(90)));

    rendering::VisualPtr quadRotor = this->scene->GetVisual("quadrotor");
    quadRotor->MoveToPositions(quadPoses, 1.0,
        boost::bind(&Movie::OnRotorComplete, this));

    this->startTime = common::Time::GetWallTime();
  }

  /*if (common::Time::GetWallTime() - this->startTime > 1.0 &&
      this->viewPoses.size() > 0)
  {
    this->userCamera->MoveToPositions(this->viewPoses, 20.0,
        boost::bind(&Movie::OnCamComplete, this));

    this->viewPoses.clear();
  }*/

  common::Time currTime = common::Time::GetWallTime();

  /*rendering::LightPtr spot1 = this->scene->GetLight("spot1");
  rendering::LightPtr spot2 = this->scene->GetLight("spot2");
  rendering::LightPtr spot3 = this->scene->GetLight("spot3");
  rendering::LightPtr spot4 = this->scene->GetLight("spot4");
  rendering::LightPtr spot5 = this->scene->GetLight("spot5");

  if (spot1 && this->spot1Pattern.size() > 0)
  {
    if (currTime - this->startTime > this->spot1Pattern.front().first)
    {
      spot1->SetDiffuseColor(this->spot1Pattern.front().second);
      this->spot1Pattern.pop_front();
    }
  }

  if (spot2 && this->spot2Pattern.size() > 0)
  {
    if (currTime - this->startTime > this->spot2Pattern.front().first)
    {
      spot2->SetDiffuseColor(this->spot2Pattern.front().second);
      this->spot2Pattern.pop_front();
    }
  }

  if (spot3 && this->spot3Pattern.size() > 0)
  {
    if (currTime - this->startTime > this->spot3Pattern.front().first)
    {
      spot3->SetDiffuseColor(this->spot3Pattern.front().second);
      this->spot3Pattern.pop_front();
    }
  }

  if (spot4 && this->spot4Pattern.size() > 0)
  {
    if (currTime - this->startTime > this->spot4Pattern.front().first)
    {
      spot4->SetDiffuseColor(this->spot4Pattern.front().second);
      this->spot4Pattern.pop_front();
    }
  }

  if (spot5 && this->spot5Pattern.size() > 0)
  {
    if (currTime - this->startTime > this->spot5Pattern.front().first)
    {
      spot5->SetDiffuseColor(this->spot5Pattern.front().second);
      this->spot5Pattern.pop_front();
    }
  }
  */
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

  poses.push_back(math::Pose(161, -92, 2, 0, 0, GZ_DTOR(90)));
  poses.push_back(math::Pose(161, -79, 2, 0, 0, GZ_DTOR(90)));
  poses.push_back(math::Pose(162, -77.7, 2, 0, 0, GZ_DTOR(0)));
  poses.push_back(math::Pose(171, -77.7, 2, 0, 0, GZ_DTOR(0)));
  poses.push_back(math::Pose(178, -75.15, 2, 0, 0, GZ_DTOR(90)));

  rendering::VisualPtr pr2 = this->scene->GetVisual("pr2");

  pr2->MoveToPositions(poses, 20.0);
  this->userCamera->TrackVisual("pr2");
}
