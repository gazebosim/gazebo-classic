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

  this->end();

  this->resizable(NULL);
}

Toolbar::~Toolbar()
{
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
      const std::vector< Geom *> *geoms;;
      std::map<std::string, Body *>::const_iterator iter;
      std::vector<Geom*>::const_iterator giter;
      std::string value;

      for (iter = bodies->begin(); iter != bodies->end(); iter++)
      {
        value = "@b@B52@s-Body:";
        this->AddToBrowser(value);
        this->AddEntityToAttributeBrowser( iter->second, "  " );

        geoms = iter->second->GetGeoms();

        for (giter = geoms->begin(); giter != geoms->end(); giter++)
        {
          value = "@b@B52@s  -Geom:";
          this->AddToBrowser(value);
          this->AddEntityToAttributeBrowser( (*giter), "    " );
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
  printf("Callback\n");
}

////////////////////////////////////////////////////////////////////////////////
// Add entity to browser
void Toolbar::AddEntityToAttributeBrowser(Entity *entity, std::string prefix)
{
  std::vector<Param*> *parameters;
  std::vector<Param*>::iterator iter;
  std::string value;
  std::string colorStr = "";

  parameters = entity->GetParameters();

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
