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

/* Desc: HTTP portal to libgazebo
 * Author: Brian Gerkey, Richard Vaughan
 * Date: 9 March 2009
 * SVN: $Id: gazebo.h 7398 2009-03-09 07:21:49Z natepak $
 */

#include "WebGazebo.hh"

#include <fstream>

#include <stdio.h>
#include <assert.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>

#include "Quatern.hh"

bool
WebGazebo::GetModelPVA(const std::string& name, 
                       websim::Time &t,
                       websim::Pose& p,
                       websim::Velocity& v,
                       websim::Acceleration& a,
                       std::string& response)
{
  // Discard any leftover responses
  this->simIface->data->responseCount = 0;

  gazebo::Pose pose;

  // Ask Gazebo
  if (!this->simIface->GetPose3d(name.c_str(), pose))
    return false;
  
  p.x = pose.pos.x;
  p.y = pose.pos.y;
  p.z = pose.pos.z;
  p.r = pose.roll;
  p.p = pose.pitch;
  p.a = pose.yaw;

  return true;
}


bool
WebGazebo::SetModelPVA(const std::string& name, 
                       const websim::Pose& p,
                       const websim::Velocity& v,
                       const websim::Acceleration& a,
                       std::string& response)
{
  // TODO: get pose, check for too-large jump, and complain about it
  // note: CheckTolerances() is a first cut at doing this

  // Tell Gazebo the new pose
  gazebo::Pose pose;
  gazebo::Vec3 lv, av, la, aa;
  pose.pos.x = p.x;
  pose.pos.y = p.y;
  pose.pos.z = p.z;
  pose.roll = p.r;
  pose.pitch = p.p;
  pose.yaw = p.a;

  lv.x = v.x;
  lv.y = v.y;
  lv.z = v.z;
  av.x = v.r;
  av.y = v.p;
  av.z = v.a;

  la.x = a.x;
  la.y = a.y;
  la.z = a.z;
  aa.x = a.r;
  aa.y = a.p;
  aa.z = a.a;

  this->simIface->SetState(name.c_str(), 
                           pose, lv, av, la, aa);

  return true;
}

bool 
WebGazebo::GetModelData(const std::string& name, 
									std::string& response,
									websim::Format format,
									void* xmlnode )
{/*
  puts("data request:");
  puts(name.c_str());
  std::string mname = name;
  int i=mname.find(".");        
  if(i>0){
  	mname.erase(i,1);
  	mname.insert(i,"-");
  }
 
  websim::Time t= GetTime();
  puts("laser:::");
  puts(mname.c_str());

  this->laserIface->Open(this->client, mname);
  uint32_t resolution;
  double fov;
  websim::Pose p;
  std::vector<double> ranges;                                                                

  p.x = laserIface->data->pose.pos.x;
  p.y = laserIface->data->pose.pos.y;
  p.z = laserIface->data->pose.pos.z;
  p.r = laserIface->data->pose.roll;
  p.p = laserIface->data->pose.pitch;
  p.a = laserIface->data->pose.yaw;
  
  //resolution = laserIface->data->res_angle;
 
  int count = laserIface->data->range_count;

  float maxRange = 0;
  for (int i = 0; i < count; i++)
  {
        ranges.push_back(laserIface->data->ranges[i]);
        
  }
  
  WebSim::GetLaserData(name, t, resolution, fov, p, ranges, format, response, xmlnode);

  this->laserIface->Close();
*/
  return true;

}

bool
WebGazebo::CheckTolerances(gazebo::Pose p, gazebo::Pose q)
{
  // Check position
  double sq_dd = ((p.pos.x - q.pos.x)*(p.pos.x - q.pos.x) +
                  (p.pos.y - q.pos.y)*(p.pos.y - q.pos.y) +
                  (p.pos.z - q.pos.z)*(p.pos.z - q.pos.z));
  if((sq_dist_tol > 0) && (sq_dd > sq_dist_tol))
    return false;

  // Check orientation
  gazebo::Quatern p_quat, q_quat;
  gazebo::Vector3 pv(p.roll, p.pitch, p.yaw);
  gazebo::Vector3 qv(q.roll, q.pitch, q.yaw);
  p_quat.SetFromEuler(pv);
  q_quat.SetFromEuler(qv);
  gazebo::Quatern da = p_quat - q_quat;
  //da.Normalize();

  double sq_da = ((da.u*da.u) +
                  (da.x*da.x) +
                  (da.y*da.y) +
                  (da.z*da.z));
  if((sq_ang_tol > 0) && (sq_da > sq_ang_tol))
    return false;

  return true;
}


bool 
WebGazebo::GetModelType(const std::string& name, std::string& type)
{
  // Discard any leftover responses
  this->simIface->data->responseCount = 0;

  // Ask Gazebo
  if (!this->simIface->GetModelType( name.c_str(), type ))
    return false;

  return true;
}

bool
WebGazebo::GetModelChildren(const std::string& model, 
				                    std::vector<std::string>& children)
{
  unsigned int num;
  std::string name;

  if (model.empty())
  {
    // Ask Gazebo
    if (!this->simIface->GetNumModels(num))
      return false;

    for (unsigned int i = 0; i < num; i++)
    {
      if (!this->simIface->GetModelName(i, name))
        return false;

      children.push_back(name);
    }
  }
  else
  {

    // Ask Gazebo
    if (!this->simIface->GetNumChildren( model.c_str(), num ))
      return false;

    for (unsigned int i = 0; i < num; i++)
    {
      if (!this->simIface->GetChildName(model.c_str(), i, name))
        return false;

      children.push_back(name);
    }
  }
  
  return true;
}

bool
WebGazebo::GetNumberOfRobots(unsigned int &n)
{
  // Discard any leftover responses
  this->simIface->data->responseCount = 0;

  /// Get the number of models 
  if (!this->simIface->GetNumModels(n))
    return false;

  return true;
}

bool WebGazebo::GetModelExtent(const std::string& name, double& bx, double& by,
                        double& bz, websim::Pose& center, std::string& response) 
{
  if (name == "sim")
  {
    bx = 0;
    by = 0;
    bz = 0;
  }
  else
  {
    gazebo::Vec3 ext;
    gazebo::Pose pose;

    if (!this->simIface->GetModelExtent(name.c_str(), ext))
    {
      response = "timeout";
      return false;
    }

    bx = ext.x;
    by = ext.y;
    bz = ext.z;

    if (!this->simIface->GetPose3d(name.c_str(), pose))
    {
      response = "timeout";
      return false;
    }

    center.x = pose.pos.x;
    center.y = pose.pos.y;
    center.z = pose.pos.z;
  }
}

