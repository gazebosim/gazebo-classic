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
/* Desc: Box geometry
 * Author: Nate Keonig, Andrew Howard
 * Date: 8 May 2003
 * CVS: $Id$
 */

#include <ode/ode.h>

#include "Body.hh"
#include "BoxGeom.hh"
#include "OgreVisual.hh"

using namespace gazebo;

//////////////////////////////////////////////////////////////////////////////
// Constructor
BoxGeom::BoxGeom(Body *body)
    : Geom(body)
{
  Param::Begin(&this->parameters);
  this->sizeP = new ParamT<Vector3>("size",Vector3(1,1,1),1);
  Param::End();
}


//////////////////////////////////////////////////////////////////////////////
// Destructor
BoxGeom::~BoxGeom()
{
  delete this->sizeP;
}

//////////////////////////////////////////////////////////////////////////////
/// Load the box
void BoxGeom::LoadChild(XMLConfigNode *node)
{
  this->sizeP->Load(node);

  // Initialize box mass matrix
  dMassSetBoxTotal(&this->mass, this->massP->GetValue(),
      this->sizeP->GetValue().x,
      this->sizeP->GetValue().y,
      this->sizeP->GetValue().z);

  // Create a box geometry with box mass matrix
  this->SetGeom(dCreateBox( 0, this->sizeP->GetValue().x, 
                            this->sizeP->GetValue().y,
                            this->sizeP->GetValue().z), true );
}

//////////////////////////////////////////////////////////////////////////////
// Set the size of the box
void BoxGeom::SetSize( Vector3 size )
{

  this->sizeP->SetValue( size );

  // Initialize box mass matrix
  dMassSetBoxTotal(&this->mass, this->massP->GetValue(), 
      this->sizeP->GetValue().x, this->sizeP->GetValue().y, 
      this->sizeP->GetValue().z);


  // Create a box geometry with box mass matrix
  this->SetGeom(dCreateBox( 0, this->sizeP->GetValue().x, 
                            this->sizeP->GetValue().y,
                            this->sizeP->GetValue().z), true );
}

//////////////////////////////////////////////////////////////////////////////
// Save the box parameters
void BoxGeom::SaveChild(std::string &prefix, std::ostream &stream)
{
  stream << prefix << *(this->sizeP) << "\n";
}
