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

#include "physics/World.hh"
#include "gazebo.h"

namespace gazebo
{
  class GUITest : public WorldPlugin
  {
    public: void Load( physics::WorldPtr &_parent, sdf::ElementPtr & /*_sdf*/ )
    {
      printf("Load gui test\n");
      this->node = transport::NodePtr(new transport::Node());
      this->node->Init(_parent->GetName());

      this->guiConfigPub = this->node->Advertise<msgs::GUIOverlayConfig>("~/gui_overlay_config");
      this->CreateGUI();
    }

    private: void CreateGUI()
    {
      msgs::GUIOverlayConfig msg;

      msg.set_layout_filename( "gui_test.layout" );
      this->guiConfigPub->Publish(msg);
    }

    private: transport::NodePtr node;
    private: transport::PublisherPtr guiConfigPub;
  };
  
  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(GUITest)

}
 
