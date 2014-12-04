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

#include "gazebo/gui/building/EditorItem.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
EditorItem::EditorItem()
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
QVector3D EditorItem::GetSize() const
{
  return QVector3D(0, 0, 0);
}

/////////////////////////////////////////////////
QVector3D EditorItem::GetScenePosition() const
{
  return QVector3D(0, 0, 0);
}

/////////////////////////////////////////////////
double EditorItem::GetSceneRotation() const
{
  return 0;
}

/////////////////////////////////////////////////
std::string EditorItem::GetType() const
{
  return this->editorType;
}

/////////////////////////////////////////////////
std::string EditorItem::GetName() const
{
  return this->name;
}

/////////////////////////////////////////////////
QColor EditorItem::Get3dColor() const
{
  return this->visual3dColor;
}

/////////////////////////////////////////////////
QString EditorItem::Get3dTexture() const
{
  return this->visual3dTexture;
}

/////////////////////////////////////////////////
void EditorItem::SetName(const std::string &_name)
{
  this->name = _name;
}

/////////////////////////////////////////////////
void EditorItem::Set3dColor(QColor _color)
{
  this->visual3dColor = _color;
  emit ColorChanged(this->visual3dColor);
}

/////////////////////////////////////////////////
void EditorItem::OnColorChanged(QColor _color)
{
  this->visual3dColor = _color;
}

/////////////////////////////////////////////////
void EditorItem::OnTextureChanged(QString _texture)
{
  this->visual3dTexture = _texture;
}

/////////////////////////////////////////////////
void EditorItem::Set3dTexture(QString _texture)
{
  this->visual3dTexture = _texture;
  emit TextureChanged(this->visual3dTexture);
}

/////////////////////////////////////////////////
void EditorItem::Set3dTransparency(float _transparency)
{
  this->visual3dTransparency = _transparency;
  emit TransparencyChanged(this->visual3dTransparency);
}

/////////////////////////////////////////////////
int EditorItem::GetLevel() const
{
  return this->level;
}

/////////////////////////////////////////////////
void EditorItem::SetLevel(int _level)
{
  this->level = _level;
  this->LevelChanged(this->level);
}

/////////////////////////////////////////////////
double EditorItem::GetLevelBaseHeight() const
{
  return this->levelBaseHeight;
}

/////////////////////////////////////////////////
void EditorItem::SetLevelBaseHeight(double _height)
{
  this->levelBaseHeight = _height;
}

/////////////////////////////////////////////////
void EditorItem::SetHighlighted(bool /*_highlighted*/)
{
}
