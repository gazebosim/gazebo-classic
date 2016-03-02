/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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

#include "gazebo/gui/building/EditorItem.hh"
#include "gazebo/gui/building/EditorItemPrivate.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
EditorItem::EditorItem() : dataPtr(new EditorItemPrivate)
{
  this->editorType = "base";
  this->name = "";

  this->level = 0;
  this->levelBaseHeight = 0;
}

/////////////////////////////////////////////////
EditorItem::~EditorItem()
{
  emit ItemDeleted();
}

/////////////////////////////////////////////////
ignition::math::Vector3d EditorItem::Size() const
{
  return ignition::math::Vector3d::Zero;
}

/////////////////////////////////////////////////
ignition::math::Vector3d EditorItem::ScenePosition() const
{
  return ignition::math::Vector3d::Zero;
}

/////////////////////////////////////////////////
double EditorItem::SceneRotation() const
{
  return 0;
}

/////////////////////////////////////////////////
std::string EditorItem::ItemType() const
{
  return this->editorType;
}

/////////////////////////////////////////////////
std::string EditorItem::Name() const
{
  return this->name;
}

/////////////////////////////////////////////////
common::Color EditorItem::Color3d() const
{
  return this->visual3dColor;
}

/////////////////////////////////////////////////
std::string EditorItem::Texture3d() const
{
  return this->visual3dTexture;
}

/////////////////////////////////////////////////
void EditorItem::SetName(const std::string &_name)
{
  this->name = _name;
}

/////////////////////////////////////////////////
void EditorItem::SetColor3d(const common::Color &_color)
{
  this->visual3dColor = _color;
  emit ColorChanged(this->visual3dColor);
}

/////////////////////////////////////////////////
void EditorItem::OnColorChanged(const common::Color &_color)
{
  this->visual3dColor = _color;
}

/////////////////////////////////////////////////
void EditorItem::SetTexture3d(const std::string &_texture)
{
  this->visual3dTexture = _texture;
  emit TextureChanged(this->visual3dTexture);
}

/////////////////////////////////////////////////
void EditorItem::OnTextureChanged(const std::string &_texture)
{
  this->visual3dTexture = _texture;
}

/////////////////////////////////////////////////
void EditorItem::Set3dTransparency(const float _transparency)
{
  this->visual3dTransparency = _transparency;
  emit TransparencyChanged(this->visual3dTransparency);
}

/////////////////////////////////////////////////
int EditorItem::Level() const
{
  return this->level;
}

/////////////////////////////////////////////////
void EditorItem::SetLevel(const int _level)
{
  this->level = _level;
  this->LevelChanged(this->level);
}

/////////////////////////////////////////////////
double EditorItem::LevelBaseHeight() const
{
  return this->levelBaseHeight;
}

/////////////////////////////////////////////////
void EditorItem::SetLevelBaseHeight(const double _height)
{
  this->levelBaseHeight = _height;
}

/////////////////////////////////////////////////
void EditorItem::SetHighlighted(const bool /*_highlighted*/)
{
}

/////////////////////////////////////////////////
int EditorItem::ZValueIdle() const
{
  return this->zValueIdle;
}

/////////////////////////////////////////////////
int EditorItem::ZValueSelected() const
{
  return this->zValueSelected;
}
