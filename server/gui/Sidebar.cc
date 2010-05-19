/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/* Desc: Sidebar
 * Author: Nate Koenig
 * Date: 13 Feb 2006
 * SVN: $Id$
 */

#include <stdio.h>
#include <FL/Fl_Value_Output.H>
#include <FL/Fl_Output.H>
#include <FL/Fl_Input.H>
#include <FL/Fl_Float_Input.H>
#include <FL/Fl_Button.H>
#include <FL/Fl_Choice.H>
#include <FL/Fl_Value_Slider.H>

#include <boost/lexical_cast.hpp>
#include <boost/bind.hpp>

#include "Gui.hh"
#include "World.hh"
#include "Body.hh"
#include "Geom.hh"
#include "Entity.hh"
#include "Light.hh"
#include "Common.hh"
#include "Model.hh"
#include "Simulator.hh"
#include "CameraManager.hh"
#include "OgreVisual.hh"
#include "OgreCamera.hh"
#include "Sidebar.hh"
#include "Global.hh"
#include "ParamBrowser.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
// Constructor
Sidebar::Sidebar(int x, int y, int w, int h, const char *l)
    : Fl_Group(x,y,w,h,l)
{
  this->box(FL_NO_BOX);
  this->color(BG_COLOR);

  this->entityBrowser = new Fl_Hold_Browser(x+10, y+20, w-20, 
      (this->h()-20)*0.25,"Models");
  this->entityBrowser->align(FL_ALIGN_TOP);
  this->entityBrowser->callback( &Sidebar::EntityBrowserCB, this );
  this->entityBrowser->color(FL_WHITE);

  x = this->entityBrowser->x() + this->entityBrowser->w()*.5 - 25;
  y = this->entityBrowser->y() + this->entityBrowser->h() + 5;
  Fl_Button *gotoButton = new Fl_Button(x,y,50,20, "Go To");
  gotoButton->callback(&Sidebar::GotoCB, this);

  this->paramColumnWidths[0] = 80;
  this->paramColumnWidths[1] = 120;
  this->paramColumnWidths[2] = 0;

  y = gotoButton->y() + gotoButton->h() + 30;
  x = this->entityBrowser->x();
  this->paramBrowser = new ParamBrowser(x, y, w-20,
      (this->h()-20)*0.5,"Parameters");
  this->paramBrowser->align(FL_ALIGN_TOP);
  //this->paramBrowser->column_char('~');
  //this->paramBrowser->column_widths( this->paramColumnWidths );
  this->paramBrowser->callback(&Sidebar::ParamBrowserCB, this);
  this->paramBrowser->color(FL_WHITE);

  y = this->paramBrowser->y() + this->paramBrowser->h() + 20;
  this->paramInput = new Fl_Float_Input(x+10, y, w-20, 20, "Param:");
  this->paramInput->align(FL_ALIGN_TOP);
  this->paramInput->labelsize(12);
  this->paramInput->when(FL_WHEN_ENTER_KEY | FL_WHEN_RELEASE | FL_WHEN_CHANGED);
  this->paramInput->callback(&Sidebar::ParamInputCB, this);
  this->paramInput->color(FL_WHITE);
  this->paramInput->value("1.0");
  this->paramInput->hide();

  this->end();

  this->resizable( this->entityBrowser );
  this->resizable( this->paramBrowser );


  World::Instance()->ConnectAddEntitySignal( 
      boost::bind(&Sidebar::AddEntityToBrowser, this, _1) );

  World::Instance()->ConnectEntitySelectedSignal( 
      boost::bind(&Sidebar::SetSelectedEntity, this, _1) );

  World::Instance()->ConnectDeleteEntitySignal( 
      boost::bind(&Sidebar::DeleteEntityFromBrowser, this, _1) );
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Sidebar::~Sidebar()
{
  delete this->paramBrowser;
  delete this->paramInput;
}

////////////////////////////////////////////////////////////////////////////////
/// Update the toolbar data
void Sidebar::Update()
{
}

////////////////////////////////////////////////////////////////////////////////
// Set the selected entity
void Sidebar::SetSelectedEntity(Entity *entity)
{
  this->paramCount = 0;

  Model *model = NULL;

  if (entity && entity->GetType() == Entity::BODY)
    model = (Model*)(entity->GetParent());
  else if (entity && entity->GetType() == Entity::MODEL)
  {
    //this->paramBrowser->deselect();
    model = (Model*)(entity);
  }

  if (model)
  {
    this->paramBrowser->Clear();
    this->Helper(model);

    // Reference
    /*for (int i=1; i <= this->entityBrowser->size(); i++)
    {
      std::string lineText = this->entityBrowser->text(i);
      if (lineText == model->GetCompleteScopedName())
        this->entityBrowser->value(i);
    }

    std::string value = "@b@B52@s@c" + model->GetTypeString();
    this->AddToParamBrowser(value);
    this->AddEntityToParamBrowser(model, "");

    const std::vector<Entity*> children = model->GetChildren();
    std::vector<Entity*>::const_iterator iter;
    
    for (iter = children.begin(); iter != children.end(); iter++)
    {
      Body *body = dynamic_cast<Body*>(*iter);

      if (!body)
        continue;

      value = std::string("@b@B52@s-") + body->GetTypeString() + ":~@b@B52@s" + body->GetName();
      this->AddToParamBrowser(value);

      if (body->GetName() == entity->GetName())
        this->paramBrowser->value(this->paramBrowser->size());

      this->AddEntityToParamBrowser( body, "  " );

      const std::vector<Entity*> children2 = body->GetCoMEntity()->GetChildren();
      std::vector<Entity*>::const_iterator iter2;

      for (iter2 = children2.begin(); iter2 != children2.end(); iter2++)
      {
        if ((*iter2)->GetType() != Entity::GEOM && 
            (*iter2)->GetType() != Entity::LIGHT)
          continue;

        value = "@b@B52@s  -" + (*iter2)->GetTypeString()+ ":~@b@B52@s" + (*iter2)->GetName();
        this->AddToParamBrowser(value);
        this->AddEntityToParamBrowser( (*iter2), "    " );

        if ((*iter2)->GetType() == Entity::GEOM)
        {
          Geom *geom = (Geom*)(*iter2);
          for (unsigned int i=0; i < geom->GetVisualCount(); i++)
          {
            OgreVisual *vis = geom->GetVisual(i);
            std::ostringstream stream;
            stream << vis->GetId();
            value = "@b@B52@s    -Visual:~@b@B52@s" + stream.str();
            this->AddToParamBrowser(value);
            this->AddEntityToParamBrowser( vis, "      " );
          }
        }
        else if ((*iter2)->GetType() == Entity::LIGHT)
        {
          Light *light = (Light*)(*iter2);

        }
      }
    }

    // Clear the remaining lines
    while ( this->paramBrowser->text(this->paramCount+1) != NULL )
    {
      this->AddToParamBrowser("");
    }
    */

    this->paramInput->hide();
  }

  if (entity && entity->GetType() == Entity::BODY)
  {
    this->paramInput->show();
    this->paramInput->value(boost::lexical_cast<std::string>(Gui::forceMultiplier).c_str());
    this->paramInput->label("Force Multiplier:");
  }


  if (!entity)
  {
    this->entityBrowser->deselect();
    //this->paramBrowser->deselect();
    //this->paramBrowser->value(0);
    //this->paramBrowser->clear();
  }
}

////////////////////////////////////////////////////////////////////////////////
// Helper
void Sidebar::Helper(const Entity *entity)
{
  if (!entity)
    return;

  const std::vector<Entity*> children = entity->GetChildren();
  std::vector<Entity*>::const_iterator iter;

  this->AddEntityToParamBrowser( entity, "    " );

  for (iter = children.begin(); iter != children.end(); iter++)
  {
    std::string value = (*iter)->GetTypeString()+ ":" + (*iter)->GetName();
    this->paramBrowser->AddDivider((*iter)->GetTypeString(), (*iter)->GetName() );
    this->Helper(*iter);
  }
}



////////////////////////////////////////////////////////////////////////////////
// Attribute browser callback
void Sidebar::ParamBrowserCB( Fl_Widget * w, void *data)
{
  Fl_Hold_Browser *browser = (Fl_Hold_Browser*)(w);
  Sidebar *toolbar = (Sidebar*)(data);
  std::string lineText, lbl;
  int beginLbl = 0;
  int endLbl = 0;
  int beginValue = 0;
  int selected = browser->value();

  if (selected <= 0)
    return;

  lineText = browser->text(selected);

  if (lineText.find("-Body") != std::string::npos)
  {
    Entity *selected = World::Instance()->GetSelectedEntity();

    beginLbl = lineText.rfind("@") + 2;

    std::string bodyName = lineText.substr(beginLbl, lineText.size()-beginLbl);

    std::string modelName = toolbar->entityBrowser->text( toolbar->entityBrowser->value() );
    std::string scopedName = modelName + "::" + bodyName;

    Entity *ent = World::Instance()->GetEntityByName(scopedName);

    if (selected != ent)
      World::Instance()->SetSelectedEntity(ent);
  }
  else if (lineText.find("-Geom") != std::string::npos)
  {
   beginLbl = lineText.rfind("@") + 2;

    std::string geomName = lineText.substr(beginLbl, lineText.size()-beginLbl);

    Entity *ent = World::Instance()->GetEntityByName(geomName);

    if (ent)
      World::Instance()->SetSelectedEntity(ent);

    return;
  }

  endLbl = lineText.find("~");
  while (lineText[beginLbl] == '@') beginLbl+=2; 
  while (lineText[beginLbl] == ' ') beginLbl++;

  beginValue = endLbl+1;
  while (lineText[beginValue] == '@') beginValue+=2; 

  /*toolbar->paramInputLbl = lineText.substr(beginLbl, endLbl-beginLbl);

  toolbar->paramInput->label(toolbar->paramInputLbl.c_str());

  toolbar->paramInput->value( lineText.substr(beginValue, lineText.size() - beginValue).c_str() );

  toolbar->paramInput->redraw();
  */
}

////////////////////////////////////////////////////////////////////////////////
// Attribute modification callback
void Sidebar::ParamInputCB( Fl_Widget *w, void *data)
{
  Fl_Input *input = (Fl_Input*)(w);

  std::string value = input->value();

  if (value.size() > 0)
  {
    double dblValue = boost::lexical_cast<double>(value);
    Gui::forceMultiplier = dblValue;
  }

  /*Fl_Input *input = (Fl_Input*)(w);
  Sidebar *toolbar = (Sidebar*)(data);
  Fl_Hold_Browser *browser = toolbar->paramBrowser;
  int selected = browser->value();
  Model *model = dynamic_cast<Model*>(World::Instance()->GetSelectedEntity());


  Body *body = NULL;
  Geom *geom = NULL;
  OgreVisual *vis = NULL;

  std::string geomName, bodyName, visNum, value, label;

  // Make sure we have a valid model
  if (!model)
  {
    gzerr(0) << "Somehow you selected something that is not a model.\n";
    return;
  }

  value = input->value();
  label = input->label();

  // Get rid of the ':' at the end
  label = label.substr(0, label.size()-1);

  // Get the name of the body and geom.
  while (selected > 0)
  {
    std::string lineText = browser->text(selected);
    int lastAmp = lineText.rfind("@")+2;

    if (lineText.find("-Geom:") != std::string::npos && geomName.empty())
      geomName = lineText.substr( lastAmp, lineText.size()-lastAmp );
    else if (lineText.find("-Body:") != std::string::npos && bodyName.empty())
      bodyName = lineText.substr( lastAmp, lineText.size()-lastAmp );
    else if (lineText.find("-Visual:") != std::string::npos && visNum.empty())
      visNum = lineText.substr( lastAmp, lineText.size()-lastAmp );
      
    selected--;
  }

  // Get the body
  if (!bodyName.empty())
    body = model->GetBody(bodyName);

  // Get the geom
  if (!geomName.empty() && body)
    geom = body->GetGeom(geomName);

  if (!visNum.empty() && geom)
    vis = geom->GetVisualById(boost::lexical_cast<int>(visNum));

  // Get the parameter
  Param *param = NULL;
  if (vis)
    param = vis->GetParam(label);
  else if (geom)
    param = geom->GetParam(label);
  else if (body)
    param = body->GetParam(label);
  else
    param = model->GetParam(label);

  if (param)
  {
    param->SetFromString( value, true );
  }
  */
}

////////////////////////////////////////////////////////////////////////////////
/// Callback for entity browser
void Sidebar::EntityBrowserCB( Fl_Widget *w, void *data )
{
  Fl_Hold_Browser *browser = (Fl_Hold_Browser*)(w);
  std::string lineText;
  int selected = browser->value();

  if (selected <=0)
    return;

  lineText = browser->text(selected);

  Entity *ent = World::Instance()->GetEntityByName(lineText);

  if (ent)
  {
    World::Instance()->SetSelectedEntity(ent);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Add entity to browser
void Sidebar::AddEntityToParamBrowser(const Common *entity, std::string prefix)
{
  std::string value;
  std::string colorStr = "";

  // Process all the parameters in the entity
  for (unsigned int i=0; i < entity->GetParamCount(); i++)
  {
    Param *param = entity->GetParam(i);
    this->paramBrowser->AddParam(param);
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Update entity browser
void Sidebar::UpdateEntityBrowser()
{
  std::vector<Model*>::const_iterator iter;
  const std::vector<Model*> models = World::Instance()->GetModels();

  this->entityBrowser->clear();
  for (iter = models.begin(); iter != models.end(); iter++)
  {
    std::vector<Entity*>::const_iterator eiter;
    for (eiter = (*iter)->GetChildren().begin(); 
         eiter != (*iter)->GetChildren().end(); eiter++)
    {
      if ((*eiter)->GetType() == Entity::MODEL)
        this->AddEntityToBrowser((Model*)*eiter);
    }

    this->AddEntityToBrowser((*iter));
  }
}

////////////////////////////////////////////////////////////////////////////////
// Add an entity to the browser
void Sidebar::AddEntityToBrowser(const Entity *entity)
{
  this->entityBrowser->add( entity->GetCompleteScopedName().c_str() );
}

////////////////////////////////////////////////////////////////////////////////
// Delete entity from browser
void Sidebar::DeleteEntityFromBrowser(const std::string &name)
{
  this->UpdateEntityBrowser();
}

////////////////////////////////////////////////////////////////////////////////
/// Joint choice callback
void Sidebar::JointChoiceCB( Fl_Widget *w, void *data )
{
  Sidebar *self = (Sidebar*)(data);
  Fl_Choice *choice = (Fl_Choice*)(w);

  Entity *entity = World::Instance()->GetSelectedEntity();
  if (entity->GetType() == Entity::MODEL)
  {
    Model *model = (Model*)(entity);

    Joint *joint = model->GetJoint( choice->text() );
    Angle lowStop = joint->GetLowStop(0);
    Angle highStop = joint->GetHighStop(0);

    self->jointForceSlider->bounds(-500,500); 
    self->jointVelocitySlider->bounds(-100,100); 
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Joint choice callback
void Sidebar::JointForceSliderCB( Fl_Widget *w, void *data )
{
  Sidebar *self = (Sidebar*)(data);
  Fl_Value_Slider *slider = (Fl_Value_Slider*)(w);

  // Only valid when a joint has been selected
  if(self->jointChoice->value() <0)
    return;

  double value = slider->value();

  Entity *entity = World::Instance()->GetSelectedEntity();
  if (entity->GetType() == Entity::MODEL)
  {
    Model *model = (Model*)(entity);

    Joint *joint = model->GetJoint( self->jointChoice->text() );
    joint->SetMaxForce( 0, value );
    joint->SetForce( 0, value );
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Joint velocity slider callback
void Sidebar::JointVelocitySliderCB( Fl_Widget *w, void *data )
{
  Sidebar *self = (Sidebar*)(data);
  Fl_Value_Slider *slider = (Fl_Value_Slider*)(w);

  // Only valid when a joint has been selected
  if(self->jointChoice->value() <0)
    return;

  double value = slider->value();

  Entity *entity = World::Instance()->GetSelectedEntity();
  if (entity->GetType() == Entity::MODEL)
  {
    Model *model = (Model*)(entity);
    Joint *joint = model->GetJoint( self->jointChoice->text() );
    joint->SetVelocity( 0, value );
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Goto callback
void Sidebar::GotoCB( Fl_Widget *w, void *data )
{

  Entity *ent = World::Instance()->GetSelectedEntity();

  if (ent)
  {
    OgreCamera *cam = CameraManager::Instance()->GetActiveCamera();
    if (cam)
      cam->MoveToEntity(ent);
  }
}
