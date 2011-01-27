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
#include "XMLConfig.hh"
#include "GazeboError.hh"
#include "Messages.hh"

using namespace gazebo;

LightMsg::LightMsg() : Message(LIGHT_MSG)
{
  this->action = LightMsg::UPDATE;
  this->type = LightMsg::POINT;
  this->castShadows = false;
}

LightMsg::LightMsg( const LightMsg &m ) : Message(LIGHT_MSG)
{
  this->id = m.id;
  this->action = m.action;
  this->type = m.type;
  this->pose = m.pose;
  this->diffuse = m.diffuse;
  this->specular = m.specular;
  this->attenuation = m.attenuation;
  this->direction = m.direction;
  this->range = m.range;
  this->castShadows = m.castShadows;
  this->spotInnerAngle = m.spotInnerAngle;
  this->spotOuterAngle = m.spotOuterAngle;
  this->spotFalloff = m.spotFalloff;
}

void LightMsg::Load(XMLConfigNode *node)
{
  std::string l_type = node->GetString("type","point",1);
  if (l_type == "point")
    this->type = POINT;
  else if (l_type == "spot")
    this->type = SPOT;
  else if (l_type == "directional")
    this->type = DIRECTIONAL;
  else
    gzthrow("Invalid light type");

  this->pose.pos = node->GetVector3("xyz",Vector3(0,0,0));  
  this->diffuse = node->GetColor("diffuse", Color(1,1,1,1));
  this->specular = node->GetColor("specular", Color(0,0,0,1));
  this->attenuation = node->GetVector3("attenuation",Vector3(.2, 0.1, 0.0));
  this->direction = node->GetVector3("direction",Vector3(0, 0, -1));
  this->range = node->GetDouble("range",20,1);
  this->castShadows = node->GetBool("castShadows",false,0);
}

VisualMsg::VisualMsg() : Message(VISUAL_MSG) 
{
  this->castShadows = true;
  this->attachAxes = true;
  this->visible = true;
  this->transparency = 0.0;
  this->size.Set(1,1,1);
}

VisualMsg::VisualMsg(const VisualMsg &m)
  : Message(m)
{
  this->parentId = m.parentId;
  this->id = m.id;
  this->action = m.action;
  this->render = m.render;
  this->mesh = m.mesh;
  this->material = m.material;
  this->castShadows = m.castShadows;
  this->attachAxes = m.attachAxes;
  this->visible = m.visible;
  this->transparency = m.transparency;
  this->points = m.points;
  this->pose = m.pose;
  this->plane = m.plane;
  this->uvTile_x = m.uvTile_x;
  this->uvTile_y = m.uvTile_y;
  this->size = m.size;
}

void VisualMsg::Load(XMLConfigNode *node)
{
  this->mesh = node->GetString("mesh","",0);
  this->material = node->GetString("material","",0);
  this->castShadows = node->GetBool("castShadows",true,0);
  this->visible = node->GetBool("visible",true,0);
  this->transparency = node->GetDouble("transparency",0.0,0);
  this->size = node->GetVector3("size", Vector3(1,1,1));
}
