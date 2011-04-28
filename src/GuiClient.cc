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
#include "common/Exception.hh"
#include "common/GazeboConfig.hh"

#include "gui/Gui.hh"
#include "transport/Transport.hh"

#include "GuiClient.hh"

using namespace gazebo;

GuiClient::GuiClient()
{
  this->quit = false;

  // load the configuration options 
  try
  {
    common::GazeboConfig::Instance()->Load();
  }
  catch (common::Exception e)
  {
    gzthrow("Error loading the Gazebo configuration file, check the .gazeborc file on your HOME directory \n" << e); 
  }

  // Start the transport system by connecting to the master.
  transport::init();

  // TODO: Move this to someplace proper
  transport::set_topic_namespace("default");
}

GuiClient::~GuiClient()
{
}

void GuiClient::Load(const std::string &filename)
{
  gui::load(filename);
}

void GuiClient::Init()
{
  gui::init();
}

void GuiClient::Run()
{
  this->transportThread = new boost::thread( &transport::run );
  gui::run();
}

void GuiClient::Quit()
{
  gui::quit();
  transport::fini();
}

