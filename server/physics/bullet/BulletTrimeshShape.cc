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
/* Desc: Trimesh geometry
 * Author: Nate Keonig
 * Date: 21 May 2009
 * SVN: $Id:$
 */

#include "BulletPhysics.hh"
#include "OgreVisual.hh"
#include "Body.hh"
#include "BulletTrimeshGeom.hh"
#include "GazeboError.hh"
#include "Simulator.hh"
#include "OgreAdaptor.hh"

using namespace gazebo;

//////////////////////////////////////////////////////////////////////////////
// Constructor
BulletTrimeshGeom::BulletTrimeshGeom(Body *body) 
  : BulletGeom(body)
{
  Param::Begin(&this->parameters);
  this->meshNameP = new ParamT<std::string>("mesh","",1);
  this->scaleP = new ParamT<Vector3>("scale",Vector3(1,1,1),0);
  Param::End();
}


//////////////////////////////////////////////////////////////////////////////
// Destructor
BulletTrimeshGeom::~BulletTrimeshGeom()
{
  delete this->meshNameP;
  delete this->scaleP;
}

//////////////////////////////////////////////////////////////////////////////
/// Update function.
void BulletTrimeshGeom::Update()
{
  BulletGeom::Update();
}

//////////////////////////////////////////////////////////////////////////////
/// Load the trimesh
void BulletTrimeshGeom::Load(XMLConfigNode *node)
{
  BulletGeom::Load(node);
}

//////////////////////////////////////////////////////////////////////////////
/// Save child parameters
void BulletTrimeshGeom::Save(std::string &prefix, std::ostream &stream)
{
  BulletGeom::Save(prefix, stream);
  stream << prefix << *(this->meshNameP) << "\n";
  stream << prefix << *(this->scaleP) << "\n";
}
 
