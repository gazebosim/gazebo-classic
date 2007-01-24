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
/* Desc: The base joint class
 * Author: Nate Keonig, Andrew Howard
 * Date: 21 May 2003
 * CVS: $Id: Joint.cc,v 1.1.2.1 2006/12/16 22:41:15 natepak Exp $
 */

#include "Joint.hh"
#include "Body.hh"


//////////////////////////////////////////////////////////////////////////////
// Constructor
Joint::Joint()
{
  this->body1 = NULL;
  this->body2 = NULL;
  return;
}


//////////////////////////////////////////////////////////////////////////////
// Desctructor
Joint::~Joint()
{
  dJointDestroy( this->jointId );
}


//////////////////////////////////////////////////////////////////////////////
// Get the type of the joint
Joint::Type Joint::GetType() const
{
  return this->type;
}



//////////////////////////////////////////////////////////////////////////////
// Get the body to which the joint is attached according the _index
Body *Joint::GetJointBody( int index ) const
{
  Body *result = NULL;

  if( index==0 || index==1 )
  {
    if (dJointGetBody( this->jointId, index ) == this->body1->GetId())
      result = this->body1;
    else
      result = this->body2; 
  }
  
  return result;
}


//////////////////////////////////////////////////////////////////////////////
// Determines of the two bodies are connected by a joint
bool Joint::AreConnected( Body *one, Body *two ) const
{
  return dAreConnected( one->GetId(), two->GetId() );
}


//////////////////////////////////////////////////////////////////////////////
// The default function does nothing. This should be overriden in the
// child classes where appropriate
double Joint::GetParam( int /*parameter*/ ) const
{
  return 0;
}


//////////////////////////////////////////////////////////////////////////////
// Make this joint a fixed joint, use this only when absolutely necessary
void Joint::SetFixed()
{
  dJointSetFixed( this->jointId );
}


//////////////////////////////////////////////////////////////////////////////
// Attach the two bodies with this joint
void Joint::Attach( Body *one, Body *two )
{
  if (!one && two)
  {
    dJointAttach( this->jointId, 0, two->GetId() );
    this->body1 = NULL;
    this->body2 = two;
  }
  else if (one && !two)
  {
    dJointAttach( this->jointId, one->GetId(), 0 );
    this->body1 = one;
    this->body2 = NULL;
  }
  else if (one && two)
  {
    dJointAttach( this->jointId, one->GetId(), two->GetId() );
    this->body1 = one;
    this->body2 = two;
  }
  return;
}


//////////////////////////////////////////////////////////////////////////////
// Detach this joint from all bodies
void Joint::Detach()
{
  dJointAttach( this->jointId, 0, 0 );  
  return;
}


//////////////////////////////////////////////////////////////////////////////
// By default this does nothing. It should be overridden in child classes 
// where appropriate
void Joint::SetParam(int /*parameter*/, double /*value*/)
{
}

