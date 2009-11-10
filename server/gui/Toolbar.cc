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
/* Desc: Toolbar
 * Author: Nate Koenig
 * Date: 13 Feb 2006
 * SVN: $Id$
 */

#include <stdio.h>
#include <FL/Fl_Value_Output.H>
#include <FL/Fl_Output.H>
#include <FL/Fl_Input.H>
#include <FL/Fl_Button.H>
#include <FL/Fl_Choice.H>
#include <FL/Fl_Value_Slider.H>

#include <boost/lexical_cast.hpp>

#include "World.hh"
#include "Body.hh"
#include "Geom.hh"
#include "Entity.hh"
#include "Common.hh"
#include "Model.hh"
#include "Simulator.hh"
#include "CameraManager.hh"
#include "OgreVisual.hh"
#include "OgreCamera.hh"
#include "Toolbar.hh"
#include "Global.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
// Constructor
Toolbar::Toolbar(int x, int y, int w, int h, const char *l)
    : Fl_Group(x,y,w,h,l)
{
  this->box(FL_UP_BOX);

  this->entityBrowser = new Fl_Hold_Browser(x+10, y+20, w-20, 25*5,"Models");
  this->entityBrowser->align(FL_ALIGN_TOP);
  this->entityBrowser->callback( &Toolbar::EntityBrowserCB, this );

  this->paramColumnWidths[0] = 80;
  this->paramColumnWidths[1] = 120;
  this->paramColumnWidths[2] = 0;

  y = this->entityBrowser->y() + this->entityBrowser->h() + 20;
  this->paramBrowser = new Fl_Hold_Browser(x+10, y, w-20,25*10,"Parameters");
  this->paramBrowser->align(FL_ALIGN_TOP);
  this->paramBrowser->column_char('~');
  this->paramBrowser->column_widths( this->paramColumnWidths );
  this->paramBrowser->callback(&Toolbar::ParamBrowserCB, this);

  y = this->paramBrowser->y() + this->paramBrowser->h() + 20;
  this->paramInput = new Fl_Input(x+10, y, w-20, 20, "Param:");
  this->paramInput->align(FL_ALIGN_TOP);
  this->paramInput->labelsize(12);
  this->paramInput->when( FL_WHEN_ENTER_KEY | FL_WHEN_RELEASE );
  this->paramInput->callback(&Toolbar::ParamInputCB, this);

  /*
  y = this->paramInput->y() + this->paramInput->h() + 20;
  this->jointChoice = new Fl_Choice(x+50, y, w-70, 20, "Joint:");
  this->jointChoice->callback(&Toolbar::JointChoiceCB, this);
  this->jointChoice->labelsize(12);

  y = this->jointChoice->y() + this->jointChoice->h() + 20;
  this->jointForceSlider = new Fl_Value_Slider(x+50, y, w-70, 20, "Force:");
  this->jointForceSlider->labelsize(12);
  this->jointForceSlider->type(FL_HOR_NICE_SLIDER);
  this->jointForceSlider->callback(&Toolbar::JointForceSliderCB, this);

  y = this->jointForceSlider->y() + this->jointForceSlider->h() + 20;
  this->jointVelocitySlider = new Fl_Value_Slider(x+50, y, w-70, 20, "Velocity:");
  this->jointVelocitySlider->labelsize(12);
  this->jointVelocitySlider->type(FL_HOR_NICE_SLIDER);
  this->jointVelocitySlider->callback(&Toolbar::JointVelocitySliderCB, this);
  */


  this->end();

  this->resizable(NULL);
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Toolbar::~Toolbar()
{
  delete this->paramBrowser;
  delete this->paramInput;
}



////////////////////////////////////////////////////////////////////////////////
/// Update the toolbar data
void Toolbar::Update()
{
  if (this->entityBrowser->size() == 0)
    this->UpdateEntityBrowser();

  Entity *entity = Simulator::Instance()->GetSelectedEntity();

  this->paramCount = 0;

  if (entity->IsModel())
  {
    Model *model = (Model*)(entity);

    std::string value = "@b@B52@s@cModel ";
    this->AddToParamBrowser(value);
    this->AddEntityToParamBrowser(model, "");

    const std::map<std::string, Body *> *bodies = model->GetBodies();
    const std::map<std::string, Geom *> *geoms;
    std::map<std::string, Body*>::const_iterator iter;
    std::map<std::string, Geom*>::const_iterator giter;

    for (unsigned int i=0; i < model->GetJointCount(); i++)
    {
      Joint *joint = model->GetJoint(i);
      this->jointChoice->add(joint->GetName().c_str(),0,0);
    }

    for (iter = bodies->begin(); iter != bodies->end(); iter++)
    {
      value = "@b@B52@s-Body:~@b@B52@s" + iter->second->GetName();
      this->AddToParamBrowser(value);
      this->AddEntityToParamBrowser( iter->second, "  " );

      geoms = iter->second->GetGeoms();

      for (giter = geoms->begin(); giter != geoms->end(); giter++)
      {
        value = "@b@B52@s  -Geom:~@b@B52@s" + giter->second->GetName();
        this->AddToParamBrowser(value);
        this->AddEntityToParamBrowser( giter->second, "    " );

        for (unsigned int i=0; i < giter->second->GetVisualCount(); i++)
        {
          OgreVisual *vis = giter->second->GetVisual(i);
          std::ostringstream stream;
          stream << vis->GetId();
          value = "@b@B52@s    -Visual:~@b@B52@s" + stream.str();
          this->AddToParamBrowser(value);
          this->AddEntityToParamBrowser( vis, "      " );
        }
      }
    }

    // Clear the remaining lines
    while ( this->paramBrowser->text(this->paramCount+1) != NULL )
    {
      this->AddToParamBrowser("");
    }

  }
}


////////////////////////////////////////////////////////////////////////////////
// Attribute browser callback
void Toolbar::ParamBrowserCB( Fl_Widget * w, void *data)
{
  Fl_Hold_Browser *browser = (Fl_Hold_Browser*)(w);
  Toolbar *toolbar = (Toolbar*)(data);
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
    /*beginLbl = lineText.rfind("@") + 2;

    std::string bodyName = lineText.substr(beginLbl, lineText.size()-beginLbl);
    std::cout << "Body Name[" << bodyName << "]\n";

    Model *model = Simulator::Instance()->GetSelectedModel();
    Body *body = model->GetBody(bodyName);
    Simulator::Instance()->SetSelectedEntity(body);
    */
  }
  else if (lineText.find("-Geom") != std::string::npos)
  {
    /*beginLbl = lineText.rfind("@") + 2;

    std::string geomName = lineText.substr(beginLbl, lineText.size()-beginLbl);
    std::cout << "Geom Name[" << geomName << "]\n";

    Model *model = Simulator::Instance()->GetSelectedModel();
    Geom *geom = model->GetGeom(geomName);
    Simulator::Instance()->SetSelectedEntity(geom);

    toolbar->paramInput->deactivate();
    Simulator::Instance()->SetSelectedEntity( );
    */
    return;
  }
  else
    toolbar->paramInput->activate();

  endLbl = lineText.find("~");
  while (lineText[beginLbl] == '@') beginLbl+=2; 
  while (lineText[beginLbl] == ' ') beginLbl++;

  beginValue = endLbl+1;
  while (lineText[beginValue] == '@') beginValue+=2; 

  toolbar->paramInputLbl = lineText.substr(beginLbl, endLbl-beginLbl);

  toolbar->paramInput->label(toolbar->paramInputLbl.c_str());

  toolbar->paramInput->value( lineText.substr(beginValue, lineText.size() - beginValue).c_str() );

  toolbar->paramInput->redraw();
}

////////////////////////////////////////////////////////////////////////////////
// Attribute modification callback
void Toolbar::ParamInputCB( Fl_Widget *w, void *data)
{
  Fl_Input *input = (Fl_Input*)(w);
  Toolbar *toolbar = (Toolbar*)(data);
  Fl_Hold_Browser *browser = toolbar->paramBrowser;
  int selected = browser->value();
  Model *model = dynamic_cast<Model*>(Simulator::Instance()->GetSelectedEntity());


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
}

////////////////////////////////////////////////////////////////////////////////
/// Callback for entity browser
void Toolbar::EntityBrowserCB( Fl_Widget *w, void *data )
{
  Fl_Hold_Browser *browser = (Fl_Hold_Browser*)(w);
  std::string lineText;
  int selected = browser->value();

  if (selected <=0)
    return;

  lineText = browser->text(selected);

  Model *model = World::Instance()->GetModelByName(lineText);
  Simulator::Instance()->SetSelectedEntity(model);
}

////////////////////////////////////////////////////////////////////////////////
// Add entity to browser
void Toolbar::AddEntityToParamBrowser(Common *entity, std::string prefix)
{
  std::vector<Param*> *parameters;
  std::vector<Param*>::iterator iter;
  std::string value;
  std::string colorStr = "";

  parameters = entity->GetParams();

  // Process all the parameters in the entity
  for (iter = parameters->begin(); iter != parameters->end(); iter++)
  {

    /*if ( i%2 == 0)
      colorStr = "@B50";
      */

    value = colorStr + "@b@s" + prefix + (*iter)->GetKey() + ":~" + 
      colorStr + "@s" + (*iter)->GetAsString();

    this->AddToParamBrowser( value );
  }
}

////////////////////////////////////////////////////////////////////////////////
// Add a line to the attribute browser
void Toolbar::AddToParamBrowser(const std::string &line)
{
  if (!this->paramBrowser->text(this->paramCount+1))
  {
    this->paramBrowser->add( line.c_str() );
  }
  else if (strcmp(this->paramBrowser->text(this->paramCount+1), line.c_str()) != 0)
  {
    this->paramBrowser->text( this->paramCount+1, line.c_str() );
  }

  this->paramCount++;
}

////////////////////////////////////////////////////////////////////////////////
/// Update entity browser
void Toolbar::UpdateEntityBrowser()
{
  std::vector<Model*>::iterator iter;
  std::vector<Model*> models = World::Instance()->GetModels();

  for (iter = models.begin(); iter != models.end(); iter++)
  {
    this->entityBrowser->add( (*iter)->GetName().c_str() );
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Joint choice callback
void Toolbar::JointChoiceCB( Fl_Widget *w, void *data )
{
  Toolbar *self = (Toolbar*)(data);
  Fl_Choice *choice = (Fl_Choice*)(w);

  Entity *entity = Simulator::Instance()->GetSelectedEntity();
  if (entity->IsModel())
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
void Toolbar::JointForceSliderCB( Fl_Widget *w, void *data )
{
  Toolbar *self = (Toolbar*)(data);
  Fl_Value_Slider *slider = (Fl_Value_Slider*)(w);

  // Only valid when a joint has been selected
  if(self->jointChoice->value() <0)
    return;

  double value = slider->value();

  Entity *entity = Simulator::Instance()->GetSelectedEntity();
  if (entity->IsModel())
  {
    Model *model = (Model*)(entity);

    Joint *joint = model->GetJoint( self->jointChoice->text() );
    joint->SetMaxForce( 0, value );
    joint->SetForce( 0, value );
    std::cout << "Set Force[" << value << "]\n";
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Joint velocity slider callback
void Toolbar::JointVelocitySliderCB( Fl_Widget *w, void *data )
{
  Toolbar *self = (Toolbar*)(data);
  Fl_Value_Slider *slider = (Fl_Value_Slider*)(w);

  // Only valid when a joint has been selected
  if(self->jointChoice->value() <0)
    return;

  double value = slider->value();

  Entity *entity = Simulator::Instance()->GetSelectedEntity();
  if (entity->IsModel())
  {
    Model *model = (Model*)(entity);
    Joint *joint = model->GetJoint( self->jointChoice->text() );
    joint->SetVelocity( 0, value );
  }
}
