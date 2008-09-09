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
 * SVN: $Id:$
 */

#include <stdio.h>
#include <FL/Fl_Value_Output.H>
#include <FL/Fl_Output.H>
#include <FL/Fl_Input.H>
#include <FL/Fl_Button.H>

#include "Body.hh"
#include "Geom.hh"
#include "Entity.hh"
#include "Model.hh"
#include "Simulator.hh"
#include "CameraManager.hh"
#include "OgreCamera.hh"
#include "Toolbar.hh"
#include "Global.hh"

using namespace gazebo;

Toolbar::Toolbar(int x, int y, int w, int h, const char *l)
    : Fl_Group(x,y,w,h,l)
{
  this->box(FL_UP_BOX);

  this->columnWidths[0] = 80;
  this->columnWidths[1] = 120;
  this->columnWidths[2] = 0;

  this->entityBrowser = new Fl_Hold_Browser(x+10, y+20, w-20, 25*10, "Attributes");
  this->entityBrowser->align(FL_ALIGN_TOP);
  this->entityBrowser->column_char('~');
  this->entityBrowser->column_widths( columnWidths );
  this->entityBrowser->callback(&Toolbar::AttributeBrowserCB, this);

  y = this->entityBrowser->y() + this->entityBrowser->h() + 20;
  this->attributeInput = new Fl_Input(x+10, y, w-20, 20, "Input:");
  this->attributeInput->align(FL_ALIGN_TOP);
  this->attributeInput->labelsize(12);
  this->attributeInput->when( FL_WHEN_ENTER_KEY | FL_WHEN_RELEASE );
  this->attributeInput->callback(&Toolbar::AttributeInputCB, this);

  this->end();

  this->resizable(NULL);
}

Toolbar::~Toolbar()
{
  delete this->attributeInput;
}

////////////////////////////////////////////////////////////////////////////////
/// Update the toolbar data
void Toolbar::Update()
{
  Entity *entity = Simulator::Instance()->GetSelectedEntity();

  this->attrCount = 0;
  if (entity)
  {
    std::string value = "@b@B52@s@cModel ";
    this->AddToBrowser(value);
    this->AddEntityToAttributeBrowser(entity, "");

    Model *model = dynamic_cast<Model *>(entity);
    if (model)
    {
      const std::map<std::string, Body *> *bodies = model->GetBodies();
      const std::map<std::string, Geom *> *geoms;
      std::map<std::string, Body*>::const_iterator iter;
      std::map<std::string, Geom*>::const_iterator giter;
      std::string value;

      for (iter = bodies->begin(); iter != bodies->end(); iter++)
      {
        value = "@b@B52@s-Body:~@b@B52@s" + iter->second->GetName();
        this->AddToBrowser(value);
        this->AddEntityToAttributeBrowser( iter->second, "  " );

        geoms = iter->second->GetGeoms();

        for (giter = geoms->begin(); giter != geoms->end(); giter++)
        {
          value = "@b@B52@s  -Geom:~@b@B52@s" + giter->second->GetName();
          this->AddToBrowser(value);
          this->AddEntityToAttributeBrowser( giter->second, "    " );
        }
      }
    }

    // Clear the remaining lines
    while ( this->entityBrowser->text(this->attrCount+1) != NULL )
    {
      this->AddToBrowser("");
    }

  }
}


////////////////////////////////////////////////////////////////////////////////
// Attribute browser callback
void Toolbar::AttributeBrowserCB( Fl_Widget * w, void *data)
{
  Fl_Hold_Browser *browser = (Fl_Hold_Browser*)(w);
  Toolbar *toolbar = (Toolbar*)(data);
  int selected = browser->value();
  std::string lineText = browser->text(selected);
  std::string lbl;
  int beginLbl = 0;
  int endLbl = 0;
  int beginValue = 0;

  if (lineText.find("-Body") != std::string::npos || 
      lineText.find("-Geom") != std::string::npos)
  {
    toolbar->attributeInput->deactivate();
    return;
  }
  else
    toolbar->attributeInput->activate();

  endLbl = lineText.find("~");
  while (lineText[beginLbl] == '@') beginLbl+=2; 
  while (lineText[beginLbl] == ' ') beginLbl++;

  beginValue = endLbl+1;
  while (lineText[beginValue] == '@') beginValue+=2; 

  toolbar->attributeInputLbl = lineText.substr(beginLbl, endLbl-beginLbl);

  toolbar->attributeInput->label(toolbar->attributeInputLbl.c_str());

  toolbar->attributeInput->value( lineText.substr(beginValue, lineText.size() - beginValue).c_str() );

  toolbar->attributeInput->redraw();
}

////////////////////////////////////////////////////////////////////////////////
// Attribute modification callback
void Toolbar::AttributeInputCB( Fl_Widget *w, void *data)
{
  Fl_Input *input = (Fl_Input*)(w);
  Toolbar *toolbar = (Toolbar*)(data);
  Fl_Hold_Browser *browser = toolbar->entityBrowser;
  int selected = browser->value();
  Model *model = dynamic_cast<Model*>(Simulator::Instance()->GetSelectedEntity());
  Body *body = NULL;
  Geom *geom = NULL;
  std::string geomName, bodyName, value, label;

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
      
    selected--;
  }

  // Get the body
  if (!bodyName.empty())
    body = model->GetBody(bodyName);

  // Get the geom
  if (!geomName.empty() && body)
    geom = body->GetGeom(geomName);

  // Get the parameter
  Param *param = NULL;
  if (geom)
    param = geom->GetParam(label);
  else if (body)
    param = body->GetParam(label);
  else
    param = model->GetParam(label);

  if (param)
  {
    param->SetFromString( value, true );
  }

  std::cout << "Label[" << label << "] Value[" << value << "]\n";
}

////////////////////////////////////////////////////////////////////////////////
// Add entity to browser
void Toolbar::AddEntityToAttributeBrowser(Entity *entity, std::string prefix)
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

    this->AddToBrowser( value );
  }
}

////////////////////////////////////////////////////////////////////////////////
// Add a line to the attribute browser
void Toolbar::AddToBrowser(const std::string &line)
{
  if (!this->entityBrowser->text(this->attrCount+1))
  {
    this->entityBrowser->add( line.c_str() );
  }
  else if (strcmp(this->entityBrowser->text(this->attrCount+1), line.c_str()) != 0)
  {
    this->entityBrowser->text( this->attrCount+1, line.c_str() );
  }

  this->attrCount++;
}
