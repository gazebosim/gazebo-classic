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
/* Desc: The base class for all physics engines
 * Author: Nate Koenig
 */

#include "msgs/msgs.h"
#include "common/Exception.hh"
#include "common/Console.hh"
#include "common/Events.hh"

#include "transport/Transport.hh"
#include "transport/Node.hh"

#include "physics/World.hh"
#include "physics/PhysicsEngine.hh"

using namespace gazebo;
using namespace physics;

////////////////////////////////////////////////////////////////////////////////
// Constructor
PhysicsEngine::PhysicsEngine(WorldPtr world)
  : world(world)
{
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(world->GetName());
  this->visPub = this->node->Advertise<msgs::Visual>("~/visual");
  this->physicsPub = this->node->Advertise<msgs::Physics>("~/physics");
  this->physicsSub = this->node->Subscribe("~/physics",
      &PhysicsEngine::OnPhysicsMsg, this);

  this->physicsRequestSub = this->node->Subscribe("~/physics_request", 
                                        &PhysicsEngine::OnPhysicsRequest, this);

  {
    /*this->visualMsg = new VisualMsg();
    this->visualMsg->parentId.clear();
    this->visualMsg->id = "physics_engine_visual";
    this->visualMsg->visible = false;
    //this->visualMsg->rtShader = false;
    this->visualMsg->castShadows = false;
    this->visualMsg->action = VisualMsg::UPDATE;
    */

    //Simulator::Instance()->SendMessage( *this->visualMsg );

    /* NATY: put this back in
    this->contactLines.resize(1);

    Material *mat = new Material();
    mat->SetName("ContactPointsMaterial");
    mat->SetPointSize(10);
    mat->SetAmbient(Color(1,0,0,1));
    mat->SetDiffuse(Color(1,0,0,1));
    mat->SetEmissive(Color(1,0,0,1));
    std::string matName = OgreCreator::CreateMaterial(mat);

    unsigned int i=0;
    for (this->contactLinesIter = this->contactLines.begin();
         this->contactLinesIter != this->contactLines.end(); 
         this->contactLinesIter++, i++)
    {
      (*this->contactLinesIter) = this->visual->AddDynamicLine(RENDERING_LINE_LIST);
      (*this->contactLinesIter)->AddPoint(math::Vector3(0,0,0));
      (*this->contactLinesIter)->AddPoint(math::Vector3(0,0,0));
      (*this->contactLinesIter)->AddPoint(math::Vector3(0,0,0));
      (*this->contactLinesIter)->AddPoint(math::Vector3(0,0,0));
      (*this->contactLinesIter)->AddPoint(math::Vector3(0,0,0));
      (*this->contactLinesIter)->AddPoint(math::Vector3(0,0,0));
      (*this->contactLinesIter)->AddPoint(math::Vector3(0,0,0));
      (*this->contactLinesIter)->AddPoint(math::Vector3(0,0,0));
      (*this->contactLinesIter)->AddPoint(math::Vector3(0,0,0));
      (*this->contactLinesIter)->AddPoint(math::Vector3(0,0,0));
      (*this->contactLinesIter)->AddPoint(math::Vector3(0,0,0));
      (*this->contactLinesIter)->AddPoint(math::Vector3(0,0,0));
      (*this->contactLinesIter)->AddPoint(math::Vector3(0,0,0));
      (*this->contactLinesIter)->AddPoint(math::Vector3(0,0,0));
      (*this->contactLinesIter)->AddPoint(math::Vector3(0,0,0));
      (*this->contactLinesIter)->AddPoint(math::Vector3(0,0,0));
      (*this->contactLinesIter)->setMaterial(matName);
    }
    */

    this->showContactConnection = event::Events::ConnectShowContactsSignal( boost::bind(&PhysicsEngine::ShowContacts, this, _1) );

    // TODO: put this back in
    //this->contactLinesIter = this->contactLines.begin();
    //delete mat;
  }

}

////////////////////////////////////////////////////////////////////////////////
// Destructor
PhysicsEngine::~PhysicsEngine()
{
  if (!this->visual.empty())
  {
    msgs::Visual msg;
    msgs::Init(msg, this->visual);
    msg.set_action( msgs::Visual::DELETE );
    this->visPub->Publish(msg);
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Return the gavity vector
math::Vector3 PhysicsEngine::GetGravity() const
{
  return this->sdf->GetOrCreateElement("gravity")->GetValueVector3("xyz");
}

////////////////////////////////////////////////////////////////////////////////
/// Add a contact visual
void PhysicsEngine::AddContactVisual(const math::Vector3 &/*pos_*/, 
                                     const math::Vector3 &/*norm_*/)
{
  // NATY: put back in
 /* if (!RenderState::GetShowContacts())
    return;

  math::Vector3 e1 = norm.GetPerpendicular(); e1.Normalize();
  math::Vector3 e2 = norm.GetCrossProd(e1); e2.Normalize();

  (*this->contactLinesIter)->SetPoint( 0, pos);
  (*this->contactLinesIter)->SetPoint( 1, pos+(norm*0.2)+(e1*0.05)+(e2*0.05));

  (*this->contactLinesIter)->SetPoint( 2, pos);
  (*this->contactLinesIter)->SetPoint( 3, pos+(norm*0.2)+(e1*0.05)-(e2*0.05));

  (*this->contactLinesIter)->SetPoint( 4, pos);
  (*this->contactLinesIter)->SetPoint( 5, pos+(norm*0.2)-(e1*0.05)+(e2*0.05));

  (*this->contactLinesIter)->SetPoint( 6, pos);
  (*this->contactLinesIter)->SetPoint( 7, pos+(norm*0.2)-(e1*0.05)-(e2*0.05));

  (*this->contactLinesIter)->SetPoint( 8, pos+(norm*0.2)+(e1*0.05)+(e2*0.05));
  (*this->contactLinesIter)->SetPoint( 9, pos+(norm*0.2)-(e1*0.05)+(e2*0.05));

  (*this->contactLinesIter)->SetPoint(10, pos+(norm*0.2)-(e1*0.05)+(e2*0.05));
  (*this->contactLinesIter)->SetPoint(11, pos+(norm*0.2)-(e1*0.05)-(e2*0.05));

  (*this->contactLinesIter)->SetPoint(12, pos+(norm*0.2)-(e1*0.05)-(e2*0.05));
  (*this->contactLinesIter)->SetPoint(13, pos+(norm*0.2)+(e1*0.05)-(e2*0.05));

  (*this->contactLinesIter)->SetPoint(14, pos+(norm*0.2)+(e1*0.05)-(e2*0.05));
  (*this->contactLinesIter)->SetPoint(15, pos+(norm*0.2)+(e1*0.05)+(e2*0.05));

  this->contactLinesIter++;

  if (this->contactLinesIter == this->contactLines.end())
    this->contactLinesIter = this->contactLines.begin();
    */
}

////////////////////////////////////////////////////////////////////////////////
// Set whether to show contacts
void PhysicsEngine::ShowContacts(const bool &show)
{
  msgs::Visual msg;
  msgs::Init(msg, this->visual);
  msg.set_visible( show );
  this->visPub->Publish(msg);

  /* TODO put back in
  if (show)
    this->contactLinesIter = this->contactLines.begin();
    */
}
