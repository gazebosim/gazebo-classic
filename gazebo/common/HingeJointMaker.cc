#include <iostream>
#include <FL/Fl.H>

#include "PhysicsEngine.hh"
#include "Joint.hh"
#include "XMLConfig.hh"
#include "Body.hh"
#include "Simulator.hh"
#include "OgreAdaptor.hh"
#include "Entity.hh"
#include "Pose3d.hh"
#include "GLWindow.hh"
#include "OgreVisual.hh"
#include "OgreCreator.hh"
#include "CameraManager.hh"
#include "OgreCamera.hh"
#include "World.hh"
#include "HingeJointMaker.hh"

using namespace gazebo;

HingeJointMaker::HingeJointMaker()
{
  this->state = 0;
  this->jointName = "";
  this->leftMousePressed = false;
}

HingeJointMaker::~HingeJointMaker()
{
}

void HingeJointMaker::Start()
{
  this->jointName = "user_hingejoint";
  this->first = NULL;
  this->second = NULL;
  this->state = 1;
}

void HingeJointMaker::Stop()
{
  this->state = 0;
}

bool HingeJointMaker::IsActive() const
{
  return this->state > 0;
}

void HingeJointMaker::MousePushCB(Vector2<int> mousePos)
{
  if (this->state == 0)
    return;

  this->leftMousePressed = false;

  this->mousePushPos = mousePos;

  std::string mod;
  OgreCamera *cam = CameraManager::Instance()->GetActiveCamera();
  this->first = OgreAdaptor::Instance()->GetEntityAt(cam, mousePos, mod);

  switch (Fl::event_button())
  {
    case FL_LEFT_MOUSE:
      this->leftMousePressed = true;
      break;
  }
}

void HingeJointMaker::MouseReleaseCB(Vector2<int> mousePos)
{
  if (this->state == 0)
    return;

  this->state++;

  std::string mod;
  OgreCamera *cam = CameraManager::Instance()->GetActiveCamera();
  this->second = OgreAdaptor::Instance()->GetEntityAt(cam, mousePos, mod);
  this->CreateTheHingeJoint();
}

void HingeJointMaker::MouseDragCB(Vector2<int> mousePos)
{
  if (this->state == 0)
    return;

  Vector3 norm;
  Vector3 p1, p2;

  p1 = this->first->GetWorldPose().pos;
  p2 = OgreAdaptor::Instance()->GetFirstContact(CameraManager::Instance()->GetActiveCamera(), mousePos);

  OgreCreator::DrawLine(p1,p2,"HingeMaker");

}

void HingeJointMaker::CreateTheHingeJoint()
{
  boost::recursive_mutex::scoped_lock lock( *Simulator::Instance()->GetMRMutex());


  Body *body1 = Simulator::Instance()->GetParentBody(this->first);
  Body *body2 = Simulator::Instance()->GetParentBody(this->second);

  Joint *joint;
  Joint::Type jtype = Joint::HINGE;

  joint = World::Instance()->GetPhysicsEngine()->CreateJoint(jtype);

  std::ostringstream newJointStr;

  newJointStr << "<?xml version='1.0'?> <gazebo:world xmlns:xi='http://www.w3.org/2001/XInclude' xmlns:gazebo='http://playerstage.sourceforge.net/gazebo/xmlschema/#gz' xmlns:model='http://playerstage.sourceforge.net/gazebo/xmlschema/#model' xmlns:sensor='http://playerstage.sourceforge.net/gazebo/xmlschema/#sensor' xmlns:body='http://playerstage.sourceforge.net/gazebo/xmlschema/#body' xmlns:geom='http://playerstage.sourceforge.net/gazebo/xmlschema/#geom' xmlns:joint='http://playerstage.sourceforge.net/gazebo/xmlschema/#joint' xmlns:interface='http://playerstage.sourceforge.net/gazebo/xmlschema/#interface' xmlns:rendering='http://playerstage.sourceforge.net/gazebo/xmlschema/#rendering' xmlns:renderable='http://playerstage.sourceforge.net/gazebo/xmlschema/#renderable' xmlns:controller='http://playerstage.sourceforge.net/gazebo/xmlschema/#controller' xmlns:physics='http://playerstage.sourceforge.net/gazebo/xmlschema/#physics' >";


  newJointStr << "<joint:hinge name=\"" << this->jointName << "\">\
    <body1>" << body1->GetCompleteScopedName() << "</body1>\
    <body2>" << body2->GetCompleteScopedName() <<  "</body2>\
    <anchor>" <<body1->GetCompleteScopedName()  << "</anchor>\
    <anchorOffset>0 0 0</anchorOffset>\
    <axis>0 0 1</axis>\
    <erp>0.8</erp>\
    <cfm>10e-5</cfm>\
    </joint:hinge>";

  newJointStr << "</gazebo:world>";


  XMLConfig *xmlConfig = new XMLConfig();

  // Load the XML tree from the given string
  try
  {
    xmlConfig->LoadString( newJointStr.str() );
  }
  catch (gazebo::GazeboError e)
  {
    gzerr(0) << "The world could not load the XML data [" << e << "]\n";
  }

  XMLConfigNode *root = xmlConfig->GetRootNode();
  joint->Load( root->GetChild() );

  OgreCreator::SetVisible("guiline", false);
}
