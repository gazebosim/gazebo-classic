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

#ifndef SDF_PHYSICS_HH
#define SDF_PHYSICS_HH

#include <string>
#include <boost/shared_ptr.hpp>
#include <iostream>

#include "sdf/interface/Param.hh"
#include "sdf/interface/color.hh"
#include "sdf/interface/pose.hh"

namespace sdf
{
  class PhysicsEngine
  {
    public: virtual void Clear() = 0;

    public: virtual void Print(const std::string &prefix) = 0;
  };
  
  class OpenDynamicsEngine : public PhysicsEngine
  {
    public: OpenDynamicsEngine() : solverType("solver", ""), dt("dt", 0.01),
            iters("iters",10), sor("sor",0.0), cfm("cfm", 0.0), erp("erp",0.0),
            contactMaxCorrectingVel("contact_max_correcting_vel",0.0),
            contactSurfaceLayer("contact_surface_layer",0.0)
            { this->Clear(); };

    public: ParamT<std::string, true> solverType;
    public: ParamT<double, true> dt;
    public: ParamT<int, true>iters;
    public: ParamT<double, false> sor;
    public: ParamT<double, false> cfm;
    public: ParamT<double, false> erp;
    public: ParamT<double, false> contactMaxCorrectingVel;
    public: ParamT<double, false> contactSurfaceLayer;

    public: void Clear()
            {
              this->solverType.Reset();
              this->dt.Reset();
              this->iters.Reset();
              this->sor.Reset();
              this->cfm.Reset();
              this->erp.Reset();
              this->contactMaxCorrectingVel.Reset();
              this->contactSurfaceLayer.Reset();
            };

    public: virtual void Print(const std::string &prefix) 
            {
              std::cout << prefix << "ODE:\n";
              std::cout << prefix << "  Solver[" << this->solverType << "]\n";
              std::cout << prefix << "  DT[" << this->dt << "]\n";
              std::cout << prefix << "  Iters[" << this->iters << "]\n";
              std::cout << prefix << "  SOR[" << this->sor << "]\n";
              std::cout << prefix << "  CFM[" << this->cfm << "]\n";
              std::cout << prefix << "  ERP[" << this->erp << "]\n";
              std::cout << prefix << "  Contact Max Correcting Vel[" << this->contactMaxCorrectingVel << "]\n";
              std::cout << prefix << "  Contact Surface Layer[" << this->contactSurfaceLayer << "]\n";
            }
  };
 
  class Physics
  {
    public: Physics() : type("type", "ode"), gravity("xyz",Vector3()) 
            { this->Clear(); };
  
    public: ParamT<std::string, true> type;
    public: ParamT<Vector3, false> gravity;

    public: boost::shared_ptr<PhysicsEngine> engine;

    public: void Clear()
    {
      this->type.Reset();
      this->gravity.Reset();
      this->engine.reset();
    }

    public: virtual void Print(const std::string &_prefix)
    {
      std::cout << _prefix << "Physics: Type[" << this->type << "] Gravity[";
      std::cout << this->gravity; 
      std::cout << "]\n";
      this->engine->Print(_prefix + "  ");
    }
  };
}

#endif
