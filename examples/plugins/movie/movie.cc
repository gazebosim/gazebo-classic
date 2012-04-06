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

Movie::Movie()
{
}

Movie::~Movie()
{
}

void Movie::Load()
{
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
  this->connections.push_back( 
      event::Events::ConnectPreRender( 
        boost::bind(&Movie::PreRender, this) ) );

  this->scene = rendering::get_scene("default");

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
}

void Movie::Init()
{
  this->userCamera = gui::get_active_camera();
  this->userCamera->SetWorldPose(this->viewPoses[0]);
}

void Movie::PreRender()
{
  if (this->startTime == common::Time(0,0))
  {
    this->startTime = common::Time::GetWallTime();
    this->userCamera->MoveToPositions(this->viewPoses, 1.0);
  }

  common::Time currTime = common::Time::GetWallTime();

  rendering::LightPtr spot1 = this->scene->GetLight("spot1");
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
}
