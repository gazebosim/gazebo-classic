#include <iostream>

#include "Camera.hh"
#include "Events.hh"
#include "MouseEvent.hh"
#include "Simulator.hh"
#include "Visual.hh"
#include "OgreCreator.hh"
#include "World.hh"
#include "CylinderMaker.hh"

using namespace gazebo;
unsigned int CylinderMaker::counter = 0;

CylinderMaker::CylinderMaker()
  : EntityMaker()
{
  this->state = 0;
  this->visual = NULL;
}

CylinderMaker::~CylinderMaker()
{
}

void CylinderMaker::Start(Scene *scene)
{
  std::ostringstream stream;
  stream <<  "user_cylinder_" << counter++;
  this->visual = new Visual(stream.str(), scene);
  this->visual->AttachMesh("unit_cylinder");
  this->state = 1;
}

void CylinderMaker::Stop()
{
  if (this->visual)
    delete this->visual;
  this->visual = NULL;

  this->state = 0;
  Events::moveModeSignal(true);
}

bool CylinderMaker::IsActive() const
{
  return this->state > 0;
}

void CylinderMaker::MousePushCB(const MouseEvent &event)
{
  if (this->state == 0)
    return;

  this->mousePushPos = event.pressPos;
}

void CylinderMaker::MouseReleaseCB(const MouseEvent &event)
{
  if (this->state == 0)
    return;

  this->state++;

  if (this->state == 3)
  {
    this->CreateTheEntity();
    this->Stop();
  }
}

void CylinderMaker::MouseDragCB(const MouseEvent &event)
{
  if (this->state == 0)
    return;

  Vector3 norm;
  Vector3 p1, p2;

  if (this->state == 1)
    norm.Set(0,0,1);
  else if (this->state == 2)
    norm.Set(1,0,0);

  p1 = event.camera->GetWorldPointOnPlane(this->mousePushPos.x, this->mousePushPos.y, norm, 0);
  p1 = this->GetSnappedPoint( p1 );

  p2 = event.camera->GetWorldPointOnPlane(event.pos.x, event.pos.y ,norm, 0);
  p2 = this->GetSnappedPoint( p2 );

  this->visual->SetPosition(p1);

  Vector3 p = this->visual->GetPosition();
  Vector3 scale;

  if (this->state == 1)
  {
    double dist = p1.Distance(p2);
    scale.x = dist*2;
    scale.y = dist*2;
    scale.z = 0.01;
  }
  else
  {
    scale = this->visual->GetScale();
   // scale.z = p2.z - p1.z;
    scale.z = (this->mousePushPos.y - event.pos.y)*0.01;
    p.z = scale.z/2.0;
  }

  this->visual->SetPosition(p);
  this->visual->SetScale(scale);
}

void CylinderMaker::CreateTheEntity()
{
  std::ostringstream newModelStr;

  if (!this->visual)
    return;

  newModelStr << "<?xml version='1.0'?> <gazebo:world xmlns:xi='http://www.w3.org/2001/XInclude' xmlns:gazebo='http://playerstage.sourceforge.net/gazebo/xmlschema/#gz' xmlns:model='http://playerstage.sourceforge.net/gazebo/xmlschema/#model' xmlns:sensor='http://playerstage.sourceforge.net/gazebo/xmlschema/#sensor' xmlns:body='http://playerstage.sourceforge.net/gazebo/xmlschema/#body' xmlns:geom='http://playerstage.sourceforge.net/gazebo/xmlschema/#geom' xmlns:joint='http://playerstage.sourceforge.net/gazebo/xmlschema/#joint' xmlns:interface='http://playerstage.sourceforge.net/gazebo/xmlschema/#interface' xmlns:rendering='http://playerstage.sourceforge.net/gazebo/xmlschema/#rendering' xmlns:renderable='http://playerstage.sourceforge.net/gazebo/xmlschema/#renderable' xmlns:controller='http://playerstage.sourceforge.net/gazebo/xmlschema/#controller' xmlns:physics='http://playerstage.sourceforge.net/gazebo/xmlschema/#physics' >";


  newModelStr << "<model:physical name=\"" << this->visual->GetName() << "\">\
    <xyz>" << this->visual->GetPosition() << "</xyz>\
    <body:cylinder name=\"body\">\
    <geom:cylinder name=\"geom\">\
    <size>" << this->visual->GetScale().x*.5 << " " << this->visual->GetScale().z << "</size>\
    <mass>0.5</mass>\
    <visual>\
    <mesh>unit_cylinder</mesh>\
    <size>" << this->visual->GetScale() << "</size>\
    <material>Gazebo/Grey</material>\
    <shader>pixel</shader>\
    </visual>\
    </geom:cylinder>\
    </body:cylinder>\
    </model:physical>";

  newModelStr <<  "</gazebo:world>";

  delete this->visual;
  this->visual = NULL;

  Simulator::Instance()->GetActiveWorld()->InsertEntity(newModelStr.str());
}

