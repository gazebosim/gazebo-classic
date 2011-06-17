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

#ifndef URDF_PHYSICS_H
#define URDF_PHYSICS_H

#include <string>
#include <tinyxml.h>
#include <boost/shared_ptr.hpp>
#include <iostream>

#include "color.h"
#include "pose.h"

namespace sdf
{
  class PhysicsEngine
  {
    public: virtual void Clear() = 0;

    public: virtual void Print() = 0;
  };
  
  class OpenDynamicsEngine : public PhysicsEngine
  {
    public: OpenDynamicsEngine() { this->Clear(); };
    public: std::string solverType;
    public: double dt;
    public: int iters;
    public: double sor;
    public: double cfm;
    public: double erp;
    public: double contactMaxCorrectingVel;
    public: double contactSurfaceLayer;

    public: void Clear()
            {
              this->solverType.clear();
              this->dt = 0;
              this->iters = 0;
              this->sor = 0;
              this->cfm = 0;
              this->erp = 0;
              this->contactMaxCorrectingVel = 0;
              this->contactSurfaceLayer = 0;
            };

    public: virtual void Print() 
            {
              std::cout << "ODE:\n";
              std::cout << "  Solver[" << this->solverType << "]\n";
              std::cout << "  DT[" << this->dt << "]\n";
              std::cout << "  Iters[" << this->iters << "]\n";
              std::cout << "  SOR[" << this->sor << "]\n";
              std::cout << "  CFM[" << this->cfm << "]\n";
              std::cout << "  ERP[" << this->erp << "]\n";
              std::cout << "  Contact Max Correcting Vel[" << this->contactMaxCorrectingVel << "]\n";
              std::cout << "  Contact Surface Layer[" << this->contactSurfaceLayer << "]\n";
            }
  };
 
  class Physics
  {
    public: Physics() { this->Clear(); };
  
    public: std::string type;
    public: Vector3 gravity;

    public: boost::shared_ptr<PhysicsEngine> engine;

    public: void Clear()
    {
      this->type.clear();
      this->gravity.Clear();
      this->engine.reset();
    }

    public: friend std::ostream &operator<<(std::ostream &out, const Physics &physics)
    {
      out << "Physics:\n";
      out << "  Type[" << physics.type << "]\n";
      out << "  Gravity[" << physics.gravity << "]\n";
      physics.engine->Print();
      return out;
    }
  };
}

#endif
