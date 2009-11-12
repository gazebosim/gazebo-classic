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
/* Desc: Trimesh shape
 * Author: Nate Keonig
 * Date: 21 May 2009
 * SVN: $Id:$
 */

#include "BulletPhysics.hh"
#include "OgreVisual.hh"
#include "Body.hh"
#include "GazeboError.hh"
#include "Simulator.hh"
#include "OgreAdaptor.hh"

#include "BulletTrimeshShape.hh"

using namespace gazebo;

//////////////////////////////////////////////////////////////////////////////
// Constructor
BulletTrimeshShape::BulletTrimeshShape(Geom *parent) 
  : TrimeshShape(parent)
{
}


//////////////////////////////////////////////////////////////////////////////
// Destructor
BulletTrimeshShape::~BulletTrimeshShape()
{
}

//////////////////////////////////////////////////////////////////////////////
/// Update function.
void BulletTrimeshShape::Update()
{
}

//////////////////////////////////////////////////////////////////////////////
/// Load the trimesh
void BulletTrimeshShape::Load(XMLConfigNode *node)
{
}
