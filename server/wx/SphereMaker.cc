#include <iostream>

#include "Camera.hh"
#include "Scene.hh"
#include "Events.hh"
#include "MouseEvent.hh"
#include "Simulator.hh"
#include "Visual.hh"
#include "World.hh"
#include "SphereMaker.hh"

using namespace gazebo;

unsigned int SphereMaker::counter = 0;

SphereMaker::SphereMaker() 
: EntityMaker()
{
  this->state = 0;
  this->visual = NULL;
}

SphereMaker::~SphereMaker()
{
}

void SphereMaker::Start(Scene *scene)
{
  std::ostringstream stream;
  stream << "user_sphere_" << counter++;
  this->visual = new Visual(stream.str(), scene);
  this->visual->AttachMesh("unit_sphere");
  this->state = 1;
}

void SphereMaker::Stop()
{
  if (this->visual)
    delete this->visual;
  this->visual = NULL;

  Events::moveModeSignal(true);
  this->state = 0;
}

bool SphereMaker::IsActive() const
{
  return this->state > 0;
}

void SphereMaker::MousePushCB(const MouseEvent &event)
{
  if (this->state == 0)
    return;

  this->mousePushPos = event.pressPos;
}

void SphereMaker::MouseReleaseCB(const MouseEvent &event)
{
  if (this->state == 0)
    return;

  this->state++;

  if (this->state == 2)
  {
    this->CreateTheEntity();
    this->Stop();
  }
}

void SphereMaker::MouseDragCB(const MouseEvent &event)
{
  if (this->state == 0)
    return;

  Vector3 norm;
  Vector3 p1, p2;

  norm.Set(0,0,1);

  p1 = event.camera->GetWorldPointOnPlane(this->mousePushPos.x, this->mousePushPos.y, norm, 0);
  p1 = this->GetSnappedPoint( p1 );

  p2 = event.camera->GetWorldPointOnPlane(event.pos.x, event.pos.y ,norm, 0);
  p2 = this->GetSnappedPoint( p2 );

  this->visual->SetPosition(p1);

  double scale = p1.Distance(p2);
  Vector3 p = this->visual->GetPosition();

  p.z = scale;

  this->visual->SetPosition(p);
  this->visual->SetScale(Vector3(scale,scale,scale));
}

void SphereMaker::CreateTheEntity()
{
  std::ostringstream newModelStr;

  if (!this->visual)
    return;

  newModelStr << "<?xml version='1.0'?> <gazebo:world xmlns:xi='http://www.w3.org/2001/XInclude' xmlns:gazebo='http://playerstage.sourceforge.net/gazebo/xmlschema/#gz' xmlns:model='http://playerstage.sourceforge.net/gazebo/xmlschema/#model' xmlns:sensor='http://playerstage.sourceforge.net/gazebo/xmlschema/#sensor' xmlns:body='http://playerstage.sourceforge.net/gazebo/xmlschema/#body' xmlns:geom='http://playerstage.sourceforge.net/gazebo/xmlschema/#geom' xmlns:joint='http://playerstage.sourceforge.net/gazebo/xmlschema/#joint' xmlns:interface='http://playerstage.sourceforge.net/gazebo/xmlschema/#interface' xmlns:rendering='http://playerstage.sourceforge.net/gazebo/xmlschema/#rendering' xmlns:renderable='http://playerstage.sourceforge.net/gazebo/xmlschema/#renderable' xmlns:controller='http://playerstage.sourceforge.net/gazebo/xmlschema/#controller' xmlns:physics='http://playerstage.sourceforge.net/gazebo/xmlschema/#physics' >";


  newModelStr << "<model:physical name=\"" << this->visual->GetName() << "\">\
    <xyz>" << this->visual->GetPosition() << "</xyz>\
    <body:sphere name=\"body\">\
    <geom:sphere name=\"geom\">\
    <size>" << this->visual->GetScale().x << "</size>\
    <mass>0.5</mass>\
    <visual>\
    <mesh>unit_sphere</mesh>\
    <size>" << this->visual->GetScale()*2 << "</size>\
    <material>Gazebo/Grey</material>\
    <shader>pixel</shader>\
    </visual>\
    </geom:sphere>\
    </body:sphere>\
    </model:physical>";

  newModelStr <<  "</gazebo:world>";

  Simulator::Instance()->GetActiveWorld()->InsertEntity(newModelStr.str());

  delete this->visual;
  this->visual = NULL;
}
