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

#include <QApplication>

#include "common/XMLConfig.hh"
#include "common/Exception.hh"
#include "rendering/Rendering.hh"
#include "gui/MainWindow.hh"
#include "gui/Gui.hh"

using namespace gazebo;

const std::string default_config =
"<?xml version='1.0'?>\
<gazebo>\
  <config>\
    <verbosity>4</verbosity>\
    <gui>\
      <size>800 600</size>\
      <pos>0 0</pos>\
    </gui>\
  </config>\
</gazebo>\
";


QApplication *g_app;
gui::MainWindow *g_main_win;

void gui::load(const std::string &filename)
{
  rendering::load(filename);
  rendering::init();

  // Load the world file
  common::XMLConfig *xmlFile = new common::XMLConfig();

  try
  {
    if (!filename.empty())
      xmlFile->Load(filename);
    else
      xmlFile->LoadString(default_config);
  }
  catch (common::Exception e)
  {
    gzthrow("The XML config file can not be loaded, please make sure is a correct file\n" << e); 
  }

  // Get the root node, and make sure we have a gazebo config
  common::XMLConfigNode *rootNode(xmlFile->GetRootNode());
  if (!rootNode || rootNode->GetName() != "gazebo")
    gzthrow("Invalid xml. Needs a root node with the <gazebo> tag");

  // Get the config node
  common::XMLConfigNode *configNode = rootNode->GetChild("config");
  if (!configNode)
    gzthrow("Invalid xml. Needs a <config> tag");
 
  // Get the gui node
  common::XMLConfigNode *guiNode = configNode->GetChild("gui");
  if (!guiNode)
    gzthrow("Invalid xml. Needs a <gui> tag");


  g_app = new QApplication(0,NULL);
  g_main_win = new MainWindow();

  g_main_win->Load(guiNode);
  g_main_win->resize(1024,768);


}

void gui::init()
{
  g_main_win->show();
  g_main_win->Init();
}

void gui::run()
{
  g_app->exec();
}

void gui::quit()
{
  g_app->quit();
}
