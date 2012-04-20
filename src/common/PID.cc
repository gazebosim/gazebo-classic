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

#include <math.h>
#include <stdio.h>
#include "PID.hh"

using namespace gazebo;
using namespace common;

/////////////////////////////////////////////////
PID::PID(double _p, double _i, double _d, double _imax, double _imin)
  : pGain(_p), iGain(_i), dGain(_d), iMax(_imax), iMin(_imin)
{
  this->pErrLast = 0.0;
  this->pErr = 0.0;
  this->iErr = 0.0;
  this->dErr = 0.0;
}

/////////////////////////////////////////////////
PID::~PID()
{
}

/////////////////////////////////////////////////
double PID::Update(double _error, common::Time _dt)
{
   double pTerm, dTerm, iTerm, cmd;
   this->pErr = _error; //this is pError = pState-pTarget
 
   if (_dt == common::Time(0,0) || isnan(_error) || isinf(_error))
     return 0.0;
 
   // Calculate proportional contribution to command
   pTerm = this->pGain * this->pErr;
 
   // Calculate the integral error
   this->iErr = this->iErr + _dt.Double() * this->pErr;
 
   //Calculate integral contribution to command
   iTerm = this->iGain * this->iErr;
 
   // Limit iTerm so that the limit is meaningful in the output
   if (iTerm > this->iMax)
   {
     iTerm = this->iMax;
     this->iErr = iTerm / this->iGain;
   }
   else if (iTerm < this->iMin)
   {
     iTerm = this->iMin;
     this->iErr = iTerm / this->iGain;
   }
 
   // Calculate the derivative error
   if (_dt != common::Time(0, 0))
   {
     this->dErr = (this->pErr - this->pErrLast) / _dt.Double();
     this->pErrLast = this->pErr;
   }

   // Calculate derivative contribution to command
   dTerm = this->dGain * this->dErr;

   printf("Err[%f] P[%f] I[%f] D[%f]\n", _error, pTerm, iTerm, dTerm);
   cmd = -pTerm - iTerm - dTerm;
 
   return cmd;
}
