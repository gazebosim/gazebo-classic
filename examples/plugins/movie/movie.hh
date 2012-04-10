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
#include <boost/bind.hpp>

#include <gui/Gui.hh>
#include <transport/TransportTypes.hh>
#include <rendering/RenderTypes.hh>
#include <common/common.h>

namespace gazebo
{

  class Movie : public SystemPlugin
  {
    public: Movie();
    public: virtual ~Movie();
    public: void Load();
    private: void Init();
    
    private: void PreRender();
    private: void OnCamComplete();
    private: void OnRotorComplete();
    private: void OnRotorLand();
    
    private: transport::NodePtr node;
    private: transport::PublisherPtr jointAnimPub;

    private: std::vector<event::ConnectionPtr> connections;
    private: rendering::UserCameraPtr userCamera;

    private: int viewPoseIndex;
    private: std::vector<math::Pose> viewPoses;

    private: rendering::ScenePtr scene;
    private: std::list<std::pair<common::Time, common::Color> > spot1Pattern;
    private: std::list<std::pair<common::Time, common::Color> > spot2Pattern;
    private: std::list<std::pair<common::Time, common::Color> > spot3Pattern;
    private: std::list<std::pair<common::Time, common::Color> > spot4Pattern;
    private: std::list<std::pair<common::Time, common::Color> > spot5Pattern;

    private: common::Time startTime;
  };
}
