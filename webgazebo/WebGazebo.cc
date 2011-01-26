/*
 * Copyright 2011 Nate Koenig & Andrew Howard
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

/* Desc: HTTP portal to libgazebo
 * Author: Brian Gerkey, Richard Vaughan
 * Date: 9 March 2009
 * SVN: $Id: gazebo.h 7398 2009-03-09 07:21:49Z natepak $
 */

#include "WebGazebo.hh"

#include <iostream>
#include <fstream>

#include <stdio.h>
#include <assert.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>

WebGazebo::WebGazebo(const std::string& fedfile,
                     const std::string& host, 
					 unsigned short port,
                     double distance_tolerance, 
					 double angle_tolerance ) : 
  websim::WebSim(host, port), 
  distance_tolerance(distance_tolerance), 
  angle_tolerance(angle_tolerance),
  goMutex(),
  goCond(),
  client(NULL),
  simIface(NULL),
  factoryIface(NULL)
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
  this->factoryIface->Open(this->client, "default");
  puts("Done.");

  puts("[webgazebo] Ready");
}

WebGazebo::~WebGazebo()
{
  // delete all open interfaces
  std::map<std::string, gazebo::Iface*>::iterator itr;
  for(itr = interfaces.begin(); itr != interfaces.end(); itr++)
  {
	itr->second->Close();
	delete itr->second;
  }

  delete this->simIface;
  delete this->factoryIface;
  delete this->client;
}
  

/** Get the current simulation time */
websim::Time 
WebGazebo::GetTime()
{
  websim::Time t;

  t.sec = (unsigned long)this->simIface->data->simTime;
  t.usec = (this->simIface->data->simTime - t.sec) * 1e6;

  return t;
}

bool
WebGazebo::GetModel(const std::string& name,
                    const std::string& type,
                    std::string& xmldata,
                    std::string& response)
{
  std::cout << "GetModel[" << name << "] type[" << type << "]\n";

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


      tag1 = "<model:physical";
      tag2 = ">";
      i = xmldata.find(tag1);
      i = xmldata.find(tag2,i);

      xmldata.insert(i+1, "<collide>none</collide><enableGravity>false</enableGravity>" );

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

  std::cout << "CreateModel name[" << name << "] type[" << type << "]\n";

  std::string xmldata;

  if(!this->GetModel(name,type,xmldata,response))
    return false;

  struct timespec sleeptime = {0, 1e08};
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
WebGazebo::ClockRunFor( uint32_t msec )
{
  unsigned int us = (unsigned int)rint(msec*1e3);
  this->simIface->Go(us, boost::bind(&WebGazebo::GoCallback, this));
  // Wait for the callback to fire
  boost::mutex::scoped_lock lock(this->goMutex);
  this->goCond.wait(lock);
  return true;
}

/*bool WebGazebo::WaitForResponse()
{
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
      return false;
    }
    nanosleep(&sleeptime, NULL);
  }

  return true;
}

*/

