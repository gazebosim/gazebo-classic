/*
 * Copyright 2012 Open Source Robotics Foundation
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

#include "EditorItem.hh"
#include "gui/qt.h"
using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
EditorItem::EditorItem()
{
  this->editorType = "base";
}

/////////////////////////////////////////////////
EditorItem::~EditorItem()
{
  emit itemDeleted();
}
/*
/////////////////////////////////////////////////
void EditorItem::AttachItem(EditorItem *_item)
{
  if (!_item->IsAttached())
  {
    _item->SetAttachedTo(this);
    this->attachedItems.push_back(_item);
  }
}

/////////////////////////////////////////////////
void EditorItem::DetachItem(EditorItem *_item)
{
  std::remove(this->attachedItems.begin(), this->attachedItems.end(),
      _item);
}

/////////////////////////////////////////////////
bool EditorItem::IsAttached()
{
}

/////////////////////////////////////////////////
void EditorItem::SetAttached(EditorItem *_parent)
{
  if (this->IsAttached())
  {
    gzerr << this->editorType << " is already attached to a parent \n";
    return;
  }
  this->parent = _parent;
}

/////////////////////////////////////////////////
EditorItem *ModelManip::GetAttachedItem(unsigned int _index)
{
  if (_index >= this->attachedObjects.size())
    gzthrow("Index too large");

  return this->attachedObjects[_index];
}

/////////////////////////////////////////////////
unsigned int EditorItem::GetAttachedItemCount()
{
  return this->attachedItems.size();
}

/////////////////////////////////////////////////
bool EditorItem::IsAttached()
{
  return (this->parent != NULL);
}
*/

/////////////////////////////////////////////////
QVector3D EditorItem::GetSize()
{
  return QVector3D(0, 0, 0);
}

/////////////////////////////////////////////////
QVector3D EditorItem::GetScenePosition()
{
  return QVector3D(0, 0, 0);
}

/////////////////////////////////////////////////
double EditorItem::GetSceneRotation()
{
  return 0;
}

/////////////////////////////////////////////////
std::string EditorItem::GetType()
{
  return this->editorType;
}
