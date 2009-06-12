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

#include <string>
#include <map>

// These headers must be included prior to the libevent headers
#include <sys/types.h>
#include <sys/queue.h>

// libgazebo;
#include "gazebo.h"

#include "websim.hh"

class WebGazebo : public websim::WebSim
{
public:
  WebGazebo(const std::string& fedfile,
	    const std::string& host, unsigned short port,
	    double dtol, double atol);
  virtual ~WebGazebo();

  bool Go(double t);
    
  // Interface to be implemented by simulators
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

  virtual bool GetLaserData(const std::string& name,
			    websim::Time& t,
			    uint32_t& resolution,
			    double& fov,
			    websim::Pose& p,
			    std::vector<double>& ranges,
			    std::string& response) { return true;}

	 
  virtual bool GetRangerData(const std::string& name,
			     websim::Time& t,
			     std::vector<websim::Pose>& p,
			     std::vector<double>& ranges,
			     std::string& response) {return true;}
  
  
  virtual bool GetModelExtent(const std::string& name,
			      double& bx,
			      double& by,
			      double& bz,
			      websim::Pose& center,
			      std::string& response) {return true;}

  virtual bool GetNumberOfRobots(unsigned int& n) {return true;}
  
  virtual bool GetSayStrings(std::vector<std::string>& sayings) {return true;}


  /** Get the current simulation time */
  virtual websim::Time GetTime();

private:
  double sq_dist_tol;
  double sq_ang_tol;

  boost::mutex goMutex;
  boost::condition goCond;

  gazebo::Client *client;
  gazebo::SimulationIface *simIface;
  gazebo::FactoryIface *factoryIface;

  // Available models
  std::map<std::string,int> models;

  bool CheckTolerances(gazebo::Pose p, gazebo::Pose q);
  bool GetModel(const std::string& name,
		const std::string& type,
		std::string& xmldata,
		std::string& response);
  void GoCallback();
};

