/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Wim Meeussen */


#include <fstream>
#include <boost/lexical_cast.hpp>
#include <algorithm>

#include "link.h"

using namespace sdf;

bool Mesh::FileExists(std::string _filename)
{
  printf("Warning: Need to implement a gazebo_config hook\n");
  std::string fullname = _filename;

  std::ifstream fin; 
  fin.open(fullname.c_str(), std::ios::in); fin.close();

  if (fin.fail()) {
    printf("Mesh [%s] does not exist\n",_filename.c_str());
    return false;
  }
  
  return true;
}

void Link::AddVisual(boost::shared_ptr<Visual> _visual)
{
  // group exists, add Visual to the vector in the map
  std::vector<boost::shared_ptr<Visual > >::iterator vis_it = find(this->visuals.begin(),this->visuals.end(), _visual);

  if (vis_it != this->visuals.end())
    printf("attempted to add a visual that already exists, skipping.\n");
  else
    this->visuals.push_back(_visual);
  printf("successfully added a new visual\n");
}

void Link::GetVisuals(std::vector<boost::shared_ptr<Visual > > &_vis) const
{
  _vis = this->visuals;
}

void Link::AddCollision(boost::shared_ptr<Collision> _collision)
{
  // group exists, add Collision to the vector in the map
  std::vector<boost::shared_ptr<Collision > >::iterator vis_it = find(this->collisions.begin(),this->collisions.end(),_collision);

  if (vis_it != this->collisions.end())
    printf("attempted to add a collision that already exists, skipping.\n");
  else
    this->collisions.push_back(_collision);

  printf("successfully added a new collision\n");
}

void Link::GetCollisions(std::vector<boost::shared_ptr<Collision > > &_col) const
{
  _col = this->collisions;
}

boost::shared_ptr<const Sensor> Link::GetSensor(const std::string& _name) const
{
  boost::shared_ptr<const Sensor> ptr;

  if (this->sensors.find(_name) == this->sensors.end())
    ptr.reset();
  else
    ptr = this->sensors.find(_name)->second;

  return ptr;
}
