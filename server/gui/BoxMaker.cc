#include <iostream>
#include <FL/Fl.H>

#include "Simulator.hh"
#include "GLWindow.hh"
#include "OgreVisual.hh"
#include "OgreCreator.hh"
#include "CameraManager.hh"
#include "OgreCamera.hh"
#include "World.hh"
#include "BoxMaker.hh"

using namespace gazebo;

BoxMaker::BoxMaker()
{
  this->state = 0;
  this->visualName = "";
  this->leftMousePressed = false;
  this->index = 0;
}

BoxMaker::~BoxMaker()
{
}

void BoxMaker::Start()
{
  std::ostringstream stream;
  std::string boxName = "user_box";

  do
  {
    stream.str("");
    stream << boxName << index;
    this->index++;
  } while (OgreCreator::Instance()->GetVisual(stream.str()));

  this->visualName = stream.str();
  this->state = 1;
}

void BoxMaker::Stop()
{
  OgreVisual *vis = OgreCreator::Instance()->GetVisual(this->visualName);
  if (vis)
    OgreCreator::Instance()->DeleteVisual(this->visualName);

  this->state = 0;
}

bool BoxMaker::IsActive() const
{
  return this->state > 0;
}

void BoxMaker::MousePushCB(Vector2<int> mousePos)
{
  if (this->state == 0)
    return;

  this->leftMousePressed = false;

  this->mousePushPos = mousePos;

  switch (Fl::event_button())
  {
    case FL_LEFT_MOUSE:
      this->leftMousePressed = true;
      break;
  }
}

void BoxMaker::MouseReleaseCB(Vector2<int> mousePos)
{
  if (this->state == 0)
    return;

  this->state++;

  if (this->state == 3)
  {
    this->CreateTheBox();
    this->Start();
  }
}

void BoxMaker::MouseDragCB(Vector2<int> mousePos)
{
  if (this->state == 0)
    return;

  Vector3 norm;
  Vector3 p1, p2;

  //if (this->state == 1)
    norm.Set(0,0,1);
  /*else if (this->state == 2)
  {
    norm = CameraManager::Instance()->GetActiveCamera()->GetDirection();
  }*/

  p1 = GLWindow::GetWorldPointOnPlane(this->mousePushPos.x, this->mousePushPos.y, norm, 0);
  p2 = GLWindow::GetWorldPointOnPlane(mousePos.x, mousePos.y ,norm, 0);

  OgreVisual *vis = NULL;
  if (OgreCreator::Instance()->GetVisual(this->visualName))
    vis = OgreCreator::Instance()->GetVisual(this->visualName);
  else
  {
    vis = OgreCreator::Instance()->CreateVisual(this->visualName);
    vis->AttachMesh("unit_box_U1V1");
    vis->SetPosition(p1);
  }

  Vector3 scale = p1-p2;
  Vector3 p = vis->GetPosition();

  if (this->state == 1)
  {
    scale.z = 0.01;
    p.x = p1.x - scale.x/2.0;
    p.y = p1.y - scale.y/2.0;
  }
  else
  {
    scale = vis->GetScale();
    scale.z = (this->mousePushPos.y - mousePos.y)*0.01;
    p.z = scale.z/2.0;
  }

  vis->SetPosition(p);

  vis->SetScale(scale);

}

void BoxMaker::CreateTheBox()
{
  boost::recursive_mutex::scoped_lock lock( *Simulator::Instance()->GetMRMutex());

  std::ostringstream newModelStr;

  OgreVisual *vis = OgreCreator::Instance()->GetVisual(this->visualName);
  if (!vis)
    return;

  newModelStr << "<?xml version='1.0'?> <gazebo:world xmlns:xi='http://www.w3.org/2001/XInclude' xmlns:gazebo='http://playerstage.sourceforge.net/gazebo/xmlschema/#gz' xmlns:model='http://playerstage.sourceforge.net/gazebo/xmlschema/#model' xmlns:sensor='http://playerstage.sourceforge.net/gazebo/xmlschema/#sensor' xmlns:body='http://playerstage.sourceforge.net/gazebo/xmlschema/#body' xmlns:geom='http://playerstage.sourceforge.net/gazebo/xmlschema/#geom' xmlns:joint='http://playerstage.sourceforge.net/gazebo/xmlschema/#joint' xmlns:interface='http://playerstage.sourceforge.net/gazebo/xmlschema/#interface' xmlns:rendering='http://playerstage.sourceforge.net/gazebo/xmlschema/#rendering' xmlns:renderable='http://playerstage.sourceforge.net/gazebo/xmlschema/#renderable' xmlns:controller='http://playerstage.sourceforge.net/gazebo/xmlschema/#controller' xmlns:physics='http://playerstage.sourceforge.net/gazebo/xmlschema/#physics' >";


  newModelStr << "<model:physical name=\"" << this->visualName << "\">\
    <xyz>" << vis->GetPosition() << "</xyz>\
    <body:box name=\"body\">\
    <geom:box name=\"geom\">\
    <size>" << vis->GetScale() << "</size>\
    <mass>0.5</mass>\
    <visual>\
    <mesh>unit_box_U1V1</mesh>\
    <size>" << vis->GetScale() << "</size>\
    <material>Gazebo/Grey</material>\
    </visual>\
    </geom:box>\
    </body:box>\
    </model:physical>";

  newModelStr <<  "</gazebo:world>";

  OgreCreator::Instance()->DeleteVisual(this->visualName);

  World::Instance()->InsertEntity(newModelStr.str());
}

