#include "XMLConfig.hh"
#include "Messages.hh"

using namespace gazebo;

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
  this->boundingbox = m.boundingbox;
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
