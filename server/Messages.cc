#include "XMLConfig.hh"
#include "Messages.hh"

using namespace gazebo;

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
}

void VisualMsg::Load(XMLConfigNode *node)
{
}
