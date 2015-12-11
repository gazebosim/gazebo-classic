/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
EditorItem::EditorItem() : editorDPtr(new EditorItemPrivate)
{
  this->Init();
}

//////////////////////////////////////////////////
EditorItem::EditorItem(EditorItemPrivate &_editorDPtr)
    : editorDPtr(&_editorDPtr)
{
  this->Init();
}

//////////////////////////////////////////////////
void EditorItem::Init()
{
  this->editorDPtr->editorType = "base";
  this->editorDPtr->name = "";

  this->editorDPtr->level = 0;
  this->editorDPtr->levelBaseHeight = 0;
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
  return this->editorDPtr->editorType;
}

/////////////////////////////////////////////////
std::string EditorItem::GetName() const
{
  return this->editorDPtr->name;
}

/////////////////////////////////////////////////
QColor EditorItem::Get3dColor() const
{
  return this->editorDPtr->visual3dColor;
}

/////////////////////////////////////////////////
QString EditorItem::Get3dTexture() const
{
  return this->editorDPtr->visual3dTexture;
}

/////////////////////////////////////////////////
void EditorItem::SetName(const std::string &_name)
{
  this->editorDPtr->name = _name;
}

/////////////////////////////////////////////////
void EditorItem::Set3dColor(QColor _color)
{
  this->editorDPtr->visual3dColor = _color;
  emit ColorChanged(this->editorDPtr->visual3dColor);
}

/////////////////////////////////////////////////
void EditorItem::OnColorChanged(QColor _color)
{
  this->editorDPtr->visual3dColor = _color;
}

/////////////////////////////////////////////////
void EditorItem::Set3dTexture(QString _texture)
{
  this->editorDPtr->visual3dTexture = _texture;
  emit TextureChanged(this->editorDPtr->visual3dTexture);
}

/////////////////////////////////////////////////
void EditorItem::OnTextureChanged(QString _texture)
{
  this->editorDPtr->visual3dTexture = _texture;
}

/////////////////////////////////////////////////
void EditorItem::Set3dTransparency(float _transparency)
{
  this->editorDPtr->visual3dTransparency = _transparency;
  emit TransparencyChanged(this->editorDPtr->visual3dTransparency);
}

/////////////////////////////////////////////////
int EditorItem::GetLevel() const
{
  return this->editorDPtr->level;
}

/////////////////////////////////////////////////
void EditorItem::SetLevel(int _level)
{
  this->editorDPtr->level = _level;
  this->LevelChanged(this->editorDPtr->level);
}

/////////////////////////////////////////////////
double EditorItem::GetLevelBaseHeight() const
{
  return this->editorDPtr->levelBaseHeight;
}

/////////////////////////////////////////////////
void EditorItem::SetLevelBaseHeight(double _height)
{
  this->editorDPtr->levelBaseHeight = _height;
}

/////////////////////////////////////////////////
void EditorItem::SetHighlighted(bool /*_highlighted*/)
{
}

/////////////////////////////////////////////////
int EditorItem::ZValueIdle() const
{
  return this->editorDPtr->zValueIdle;
}

/////////////////////////////////////////////////
int EditorItem::ZValueSelected() const
{
  return this->editorDPtr->zValueSelected;
}
