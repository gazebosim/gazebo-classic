#include <iostream>

#include "Events.hh"
#include "MouseEvent.hh"
#include "Simulator.hh"
#include "Visual.hh"
#include "Camera.hh"
#include "Scene.hh"
#include "Visual.hh"
#include "World.hh"
#include "BoxMaker.hh"

using namespace gazebo;

unsigned int BoxMaker::counter = 0;

BoxMaker::BoxMaker() 
: EntityMaker()
{
  this->state = 0;
  this->visual = NULL;
}

BoxMaker::~BoxMaker()
{
}

void BoxMaker::Start(Scene *scene)
{
  std::ostringstream stream;
  stream << "user_box_" << counter++;
  this->visual = new Visual(stream.str(), scene );
  this->visual->AttachMesh("unit_box_U1V1");

  this->state = 1;
}

void BoxMaker::Stop()
{
  if (this->visual)
    delete this->visual;
  this->visual = NULL;
  this->state = 0;
  Events::moveModeSignal(true);
}

bool BoxMaker::IsActive() const
{
  return this->state > 0;
}

void BoxMaker::MousePushCB(const MouseEvent &event)
{
  if (this->state == 0)
    return;

  this->mousePushPos = event.pressPos;
}

void BoxMaker::MouseReleaseCB(const MouseEvent &event)
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

void BoxMaker::MouseDragCB(const MouseEvent &event)
{
  if (this->state == 0)
    return;

  Vector3 norm(0,0,1);
  Vector3 p1, p2;

  p1 = event.camera->GetWorldPointOnPlane(this->mousePushPos.x, this->mousePushPos.y, norm, 0);
  p1 = this->GetSnappedPoint( p1 );

  p2 = event.camera->GetWorldPointOnPlane(event.pos.x, event.pos.y ,norm, 0);
  p2 = this->GetSnappedPoint( p2 );

  this->visual->SetPosition(p1);

  Vector3 scale = p1-p2;
  Vector3 p = this->visual->GetPosition();

  if (this->state == 1)
  {
    scale.z = 0.01;
    p.x = p1.x - scale.x/2.0;
    p.y = p1.y - scale.y/2.0;
  }
  else
  {
    scale = this->visual->GetScale();
    scale.z = (this->mousePushPos.y - event.pos.y)*0.01;
    p.z = scale.z/2.0;
  }

  this->visual->SetPosition(p);

  this->visual->SetScale(scale);
}

void BoxMaker::CreateTheEntity()
{
  std::ostringstream newModelStr;

  if (!this->visual)
    return;

  newModelStr << "<?xml version='1.0'?> <gazebo:world xmlns:xi='http://www.w3.org/2001/XInclude' xmlns:gazebo='http://playerstage.sourceforge.net/gazebo/xmlschema/#gz' xmlns:model='http://playerstage.sourceforge.net/gazebo/xmlschema/#model' xmlns:sensor='http://playerstage.sourceforge.net/gazebo/xmlschema/#sensor' xmlns:body='http://playerstage.sourceforge.net/gazebo/xmlschema/#body' xmlns:geom='http://playerstage.sourceforge.net/gazebo/xmlschema/#geom' xmlns:joint='http://playerstage.sourceforge.net/gazebo/xmlschema/#joint' xmlns:interface='http://playerstage.sourceforge.net/gazebo/xmlschema/#interface' xmlns:rendering='http://playerstage.sourceforge.net/gazebo/xmlschema/#rendering' xmlns:renderable='http://playerstage.sourceforge.net/gazebo/xmlschema/#renderable' xmlns:controller='http://playerstage.sourceforge.net/gazebo/xmlschema/#controller' xmlns:physics='http://playerstage.sourceforge.net/gazebo/xmlschema/#physics' >";


  newModelStr << "<model:physical name=\"" << this->visual->GetName() << "\">\
    <xyz>" << this->visual->GetPosition() << "</xyz>\
    <body:box name=\"body\">\
    <geom:box name=\"geom\">\
    <size>" << this->visual->GetScale() << "</size>\
    <mass>0.5</mass>\
    <visual>\
    <mesh>unit_box_U1V1</mesh>\
    <size>" << this->visual->GetScale() << "</size>\
    <material>Gazebo/Grey</material>\
    <shader>pixel</shader>\
    </visual>\
    </geom:box>\
    </body:box>\
    </model:physical>";

  newModelStr <<  "</gazebo:world>";

  delete this->visual;
  this->visual = NULL;

  Simulator::Instance()->GetActiveWorld()->InsertEntity(newModelStr.str());
}
