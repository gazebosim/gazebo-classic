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

/* Author: Nate Koenig */


#include <fstream>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <algorithm>

#include "parser.h"
#include "physics.h"

using namespace sdf;

bool OpenDynamicsEngine::InitXml(TiXmlElement *_config)
{
  TiXmlElement *odeConfig = _config->FirstChildElement("ode");

  if ( !odeConfig )
  {
    printf("Error: Physics element missing <ode>\n");
    return false;
  }

  TiXmlElement *solverConfig = odeConfig->FirstChildElement("solver");
  if (!solverConfig)
  {
    printf("Error: ODE Physics missing solver element\n");
    return false;
  }

  this->solverType = solverConfig->Attribute("type");
  if (this->solverType.empty())
  {
    printf("Error: ODE Physics missing solver type\n");
    return false;
  }

  std::string dtStr = solverConfig->Attribute("dt");
  if (dtStr.empty())
  {
    printf("Error: ODE Physics solver missing dt attribute\n");
    return false;
  }
  if (!getDoubleFromStr(dtStr, this->dt))
  {
    printf("Error: ODE Physics solver malformed dt attribute\n");
    return false;
  }

  std::string itersStr = solverConfig->Attribute("iters");
  if (itersStr.empty())
  {
    printf("Error: ODE Physics solver missing iters attribute\n");
    return false;
  }
  if (!getIntFromStr(itersStr, this->iters))
  {
    printf("Error: ODE Physics solver malformed iters attribute\n");
    return false;
  }

  std::string sorStr = solverConfig->Attribute("sor");
  if (sorStr.empty())
  {
    printf("Error: ODE Physics solver missing sor attribute\n");
    return false;
  }
  if (!getDoubleFromStr(sorStr, this->sor))
  {
    printf("Error: ODE Physics solver malformed sor attribute\n");
    return false;
  }


  // Contraints
  TiXmlElement *constraintsConfig = odeConfig->FirstChildElement("constraints");
  if (constraintsConfig)
  {
    std::string cfmStr = constraintsConfig->Attribute("cfm");
    if (cfmStr.empty())
    {
      printf("Error: ODE Physics contraints missing cfm attribute\n");
      return false;
    }
    if (!getDoubleFromStr(cfmStr, this->cfm))
    {
      printf("Error: ODE Physics contraints malformed cfm attribute\n");
      return false;
    }

    std::string erpStr = constraintsConfig->Attribute("erp");
    if (erpStr.empty())
    {
      printf("Error: ODE Physics contraints missing erp attribute\n");
      return false;
    }
    if (!getDoubleFromStr(erpStr, this->erp))
    {
      printf("Error: ODE Physics contraints malformed erp attribute\n");
      return false;
    }

    std::string contactMaxCorrectingVelStr = constraintsConfig->Attribute("contact_max_correcting_vel");
    if (contactMaxCorrectingVelStr.empty())
    {
      printf("Error: ODE Physics contraints missing contact_max_correcting_vel attribute\n");
      return false;
    }
    if (!getDoubleFromStr(contactMaxCorrectingVelStr, this->contactMaxCorrectingVel))
    {
      printf("Error: ODE Physics contraints malformed contact_max_correcting_vel attribute\n");
      return false;
    } 
 
    std::string contactSurfaceLayerStr = constraintsConfig->Attribute("contact_surface_layer");
    if (contactSurfaceLayerStr.empty())
    {
      printf("Error: ODE Physics contraints missing contact_surface_layer attribute\n");
      return false;
    }
    if (!getDoubleFromStr(contactSurfaceLayerStr, this->contactSurfaceLayer))
    {
      printf("Error: ODE Physics contraints malformed contact_surface_layer attribute\n");
      return false;
    }
  }

  return true;
}

////////////////////////////////////////////////////////////////////////////////
bool Physics::InitXml(TiXmlElement* _config)
{
  this->Clear();
  
  if (!_config)
  {
    printf("Error: xml config is NULL\n");
    return false;
  }

  this->type = _config->Attribute("type");
  if (this->type.empty())
  {
    printf("Error: Missing physics type attribute\n");
    return false;
  }

  TiXmlElement *gravityElement = _config->FirstChildElement("gravity");
  if (gravityElement)
  {
    if (!this->gravity.Init(gravityElement->Attribute("xyz")))
    {
      printf("Gravity has malformed xyz\n");
      this->gravity.Clear();
      return false;
    }
  }

  if (this->type == "ode")
  {
    this->engine.reset(new OpenDynamicsEngine );
  }
  else
  {
    printf("Error: Unknown physics engine type[%s]\n",this->type.c_str());
    return false;
  }
  this->engine->InitXml(_config);
  
  return true;
}
