/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

#include "gazebo/gui/qt.h"
#include "gazebo/gui/GuiTypes.hh"
#include "gazebo/gui/viewers/ViewFactory.hh"

#include "gazebo/gui/viewers/ImageView.hh"
#include "gazebo/gui/viewers/ImagesView.hh"
#include "gazebo/gui/viewers/LaserView.hh"
#include "gazebo/gui/viewers/TextView.hh"

void RegisterImageView();
void RegisterImagesView();
void RegisterLaserView();

using namespace gazebo;
using namespace gui;

std::map<std::string, ViewFactoryFn> ViewFactory::viewMap;

/////////////////////////////////////////////////
void ViewFactory::RegisterAll()
{
  RegisterLaserView();
  RegisterImageView();
  RegisterImagesView();
}

/////////////////////////////////////////////////
void ViewFactory::RegisterView(const std::string &_msgType,
                               ViewFactoryFn _factoryfn)
{
  viewMap[_msgType] = _factoryfn;
}

/////////////////////////////////////////////////
TopicView *ViewFactory::NewView(const std::string &_msgType,
                                const std::string &_topicName,
                                QWidget *_parent)
{
  TopicView *view = NULL;

  std::map<std::string, ViewFactoryFn>::iterator iter = viewMap.find(_msgType);
  if (iter != viewMap.end())
    view = (iter->second) (_parent);
  else
    view = new TextView(_parent, _msgType);

  view->SetTopic(_topicName);

  return view;
}

/////////////////////////////////////////////////
void ViewFactory::GetViewTypes(std::vector<std::string> &_types)
{
  _types.clear();

  std::map<std::string, ViewFactoryFn>::const_iterator iter;
  for (iter = viewMap.begin(); iter != viewMap.end(); ++iter)
  {
    _types.push_back(iter->first);
  }
}
