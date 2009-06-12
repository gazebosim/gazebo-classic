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

  // Ask Gazebo
  this->simIface->GetPose3d(name.c_str());
  
  // Wait for the response
  double timeout = 3.0;
  struct timeval t0, t1;
  gettimeofday(&t0, NULL);
  struct timespec sleeptime = {0, 1000000};
  while(this->simIface->data->responseCount == 0)
  {
    gettimeofday(&t1, NULL);
    if(((t1.tv_sec + t1.tv_usec/1e6) - (t0.tv_sec + t0.tv_usec/1e6)) 
       > timeout)
    {
      response= "Timeout";
      return false;
    }
    nanosleep(&sleeptime, NULL);
  }

  assert(this->simIface->data->responseCount == 1);
  p.x = this->simIface->data->responses[0].modelPose.pos.x;
  p.y = this->simIface->data->responses[0].modelPose.pos.y;
  p.z = this->simIface->data->responses[0].modelPose.pos.z;
  p.r = this->simIface->data->responses[0].modelPose.roll;
  p.p = this->simIface->data->responses[0].modelPose.pitch;
  p.a = this->simIface->data->responses[0].modelPose.yaw;

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

