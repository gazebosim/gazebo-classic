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
 * Author: Brian Gerkey, Richard Vaughan, Nate Koenig, Abbas Sadat
 * Date: 9 March 2009
 * SVN: $Id: gazebo.h 7398 2009-03-09 07:21:49Z natepak $
 */

#include <gazebo.h>
#include <websim.hh>

class WebGazebo : public websim::WebSim
{
public:
  WebGazebo(const std::string& fedfile,
	    const std::string& host, unsigned short port,
	    double dtol, double atol);
  virtual ~WebGazebo();

  bool Go(double t);
  
  // start WebSim Interface ===================================================
  
  virtual std::string IdentificationString()
  { return "WebGazebo"; }
  
  virtual std::string VersionString()
  {  return "0.1"; }
  
  virtual bool CreateModel(const std::string& name, 
			   const std::string& type,
			   std::string& error);

  virtual bool DeleteModel(const std::string& name,
			   std::string& error);

  virtual bool SetModelPVA(const std::string& name, 
			   const websim::Pose& p,
			   const websim::Velocity& v,
			   const websim::Acceleration& a,
			   std::string& error);

  virtual bool GetModelPVA(const std::string& name, 
			   websim::Time &t,
			   websim::Pose& p,
			   websim::Velocity& v,
			   websim::Acceleration& a,
			   std::string& error);

  virtual bool GetModelType(const std::string& name, std::string& type);

  virtual bool GetModelData(const std::string& name, 
			    std::string& response,
			    websim::Format format,
			    void* xmlnode );
  
  virtual bool GetModelChildren(const std::string& model, 
				std::vector<std::string>& children);
  
  virtual bool GetModelGeometry(const std::string& name,
			      double& bx,
			      double& by,
			      double& bz,
			      websim::Pose& center,
			      std::string& response);

  virtual websim::Time GetTime();

  virtual bool ClockStart() { simIface->Unpause(); return true;}

  virtual bool ClockStop() { simIface->Pause(); return true;}

  virtual bool ClockRunFor( uint32_t msec );

  // end WebSim Interface ====================================================


private: // all private members are specific to WebGazebo


  //bool WaitForResponse();

  double distance_tolerance;
  double angle_tolerance;
  
  boost::mutex goMutex;
  boost::condition goCond;
  
  gazebo::Client *client;
  gazebo::SimulationIface *simIface;
  gazebo::FactoryIface *factoryIface;

  std::map<std::string,gazebo::Iface*> interfaces;

  // Available models
  std::map<std::string,int> models;

  //bool CheckTolerances(gazebo::Pose p, gazebo::Pose q);

  bool GetModel(const std::string& name,
		const std::string& type,
		std::string& xmldata,
		std::string& response);

  void GoCallback();
};

