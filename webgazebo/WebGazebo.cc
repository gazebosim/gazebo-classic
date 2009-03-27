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
 * Author: Brian Gerkey
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

#include "../server/Quatern.hh"

// TODO:
//   - make ghost models not collide, fall, etc.

WebGazebo::WebGazebo(const std::string& fedfile,
                     const std::string& host, unsigned short port,
                     double dtol, double atol) :
        websim::WebSim(fedfile, host, port), 
        sq_dist_tol(dtol*dtol), sq_ang_tol(atol*atol)
{
  // Hook up to Gazebo
  printf("[webgazebo] Opening Gazebo simulation interface...");
  fflush(stdout);
  this->client = new gazebo::Client();
  this->simIface = new gazebo::SimulationIface();
  this->factoryIface = new gazebo::FactoryIface();
  this->client->ConnectWait(0, GZ_CLIENT_ID_USER_FIRST);
  // Open the Simulation Interface; let exceptions leak out
  this->simIface->Open(this->client, "default");
  this->factoryIface->Open(this->client, "factory_iface");
  puts("Done.");

  puts("[webgazebo] Ready");
}

WebGazebo::~WebGazebo()
{
  delete this->simIface;
  delete this->client;
}

bool
WebGazebo::GetModelPVA(const std::string& name, 
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

bool
WebGazebo::GetModel(const std::string& name,
                    const std::string& type,
                    std::string& xmldata,
                    std::string& response)
{
  std::vector<std::string> gmpath_parts;
  // Search GAZEBO_MODEL_PATH for a matching .model file
  const char* gmpath = getenv("GAZEBO_MODEL_PATH");
  if(gmpath)
  {
    StringSplit(gmpath, gmpath_parts, ":");
  }
  gmpath_parts.push_back(std::string(INSTALL_PREFIX) + "/share/gazebo/worlds/models");
  for(unsigned int i=0; i<gmpath_parts.size(); i++)
  {
    std::string p = gmpath_parts[0] + "/" + type + ".model";
    if(access(p.c_str(), F_OK) == 0)
    {
      std::ifstream ifs(p.c_str());
      xmldata = std::string((std::istreambuf_iterator<char>(ifs)), 
                            std::istreambuf_iterator<char>());

      // HACK: rename with a VERY crude textual replacement
      std::string tag1 = "<model:physical";
      std::string tag2 = "name=\"";
      int i = xmldata.find(tag1);
      if(i < 0)
        return false;
      i = xmldata.find(tag2, i);
      if(i < 0)
        return false;
      int j = xmldata.find("\"", i+tag2.length());
      xmldata.replace(i,j-i+1,std::string("name=\"" + name + "\""));
      return true;
    }
  }

  response = "ERROR: Could not find file " + type + ".model. Searched in:\n";
  for(unsigned int i=0; i<gmpath_parts.size(); i++)
    response += "  " + gmpath_parts[i] + "\n";
  return false;
}

bool
WebGazebo::CreateModel(const std::string& name, 
                       const std::string& type,
                       std::string& response)
{
  std::string xmldata;
  if(!GetModel(name,type,xmldata,response))
    return false;

  struct timespec sleeptime = {0, 100000000};
  for(;;)
  {
    this->factoryIface->Lock(1);
    if(!strcmp((const char*)this->factoryIface->data->newModel,""))
    {
      strcpy((char*)this->factoryIface->data->newModel, xmldata.c_str());
      this->factoryIface->Unlock();
      break;
    }
    else
    {
      this->factoryIface->Unlock();
      nanosleep(&sleeptime, NULL);
    }
  }

  // Now, wait until Gazebo has consumed the new model
  // For some reason, this hangs forever.
  /*
  for(;;)
  {
    this->factoryIface->Lock(1);
    if(!strcmp((const char*)this->factoryIface->data->newModel,""))
    {
      this->factoryIface->Unlock();
      break;
    }
    else
    {
      this->factoryIface->Unlock();
      nanosleep(&sleeptime, NULL);
    }
  }
  */

  response = std::string("Created model ") + name + " of type " + type;
  return true;
}

bool
WebGazebo::DeleteModel(const std::string& name, 
                       std::string& response)
{
  response = "DeleteModel not implemented";
  return false;
}

void
WebGazebo::GoCallback()
{
  this->goCond.notify_one();
}

bool
WebGazebo::Go(double t)
{
  unsigned int us = (unsigned int)rint(t*1e6);
  this->simIface->Go(us,
                     boost::bind(&WebGazebo::GoCallback, this));
  // Wait for the callback to fire
  boost::mutex::scoped_lock lock(this->goMutex);
  this->goCond.wait(lock);
  return true;
}
