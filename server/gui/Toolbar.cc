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
#include <FL/Fl_Button.H>

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

  this->entityBrowser = new Fl_Hold_Browser(x+10, y+20, w-20, 25*5, "Attributes");
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
  std::vector<Param*> *parameters;
  std::vector<Param*>::iterator iter;

  if (entity)
  {
    //Model *model = dynamic_cast<Model *>(entity);

    parameters = entity->GetParameters();

    int i=0;
    for (iter = parameters->begin(); iter != parameters->end(); iter++, i++)
    {
      std::string value;
      //boost::any anyValue = (*iter)->Get();
      //std::string typeName = (*iter)->GetTypename();
      std::string colorStr = "";

      /*if ( i%2 == 0)
        colorStr = "@B50";
        */

      value = colorStr + "@b@s" + (*iter)->GetKey() + ":~" + colorStr + "@s";
      value += (*iter)->GetAsString();


      // Convert the variable value to a string
      /*if (typeName == typeid(float).name())
        value += boost::lexical_cast<std::string>(
            boost::any_cast<float>( anyValue ));
      else if (typeName == typeid(double).name())
        value += boost::lexical_cast<std::string>(
            boost::any_cast<double>( anyValue )); 
      else if (typeName == typeid(int).name())
        value += boost::lexical_cast<std::string>(
            boost::any_cast<int>( anyValue ));
      else if (typeName == typeid(bool).name())
        value += boost::lexical_cast<std::string>( 
            boost::any_cast<bool>(anyValue) );
      else if (typeName == typeid(long).name())
        value += boost::lexical_cast<std::string>( 
            boost::any_cast<long>(anyValue) );
      else if (typeName == typeid(Quatern).name())
        value += boost::lexical_cast<std::string>(
            boost::any_cast<Quatern>( anyValue ));
      else if (typeName == typeid(Vector3).name())
        value += boost::lexical_cast<std::string>(
            boost::any_cast<Vector3>( anyValue ));
      else if (typeName == typeid(std::string).name())
        value += boost::any_cast<std::string>( anyValue );
      else
        gzerr(0) << "Unknown typename[" << typeName << "]\n";
        */

      if (!this->entityBrowser->text(i+1))
      {
        this->entityBrowser->add( value.c_str() );
      }
      else if (strcmp(this->entityBrowser->text(i+1), value.c_str()) != 0)
      {
        this->entityBrowser->text( i+1, value.c_str() );
      }
    }

    // Clear the remaining lines
    while ( this->entityBrowser->text(i+1) != NULL )
    {
      this->entityBrowser->text( i+1, "" );
      i++;
    }

  }
}


////////////////////////////////////////////////////////////////////////////////
// Attribute browser callback
void Toolbar::AttributeBrowserCB( Fl_Widget * w, void *data)
{
  printf("Callback\n");
}
