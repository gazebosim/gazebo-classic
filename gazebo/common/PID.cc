/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#include <cmath>
#include <stdio.h>

#include "gazebo/math/Helpers.hh"
#include "PID.hh"

using namespace gazebo;
using namespace common;

/////////////////////////////////////////////////
PID::PID(double _p, double _i, double _d, double _imax, double _imin,
         double _cmdMax, double _cmdMin)
  : pGain(_p), iGain(_i), dGain(_d), iMax(_imax), iMin(_imin),
    cmdMax(_cmdMax), cmdMin(_cmdMin)
{
  this->Reset();
}

/////////////////////////////////////////////////
PID::~PID()
{
}

/////////////////////////////////////////////////
void PID::Init(double _p, double _i, double _d, double _imax, double _imin,
         double _cmdMax, double _cmdMin)
{
  this->pGain = _p;
  this->iGain = _i;
  this->dGain = _d;
  this->iMax = _imax;
  this->iMin = _imin;
  this->cmdMax = _cmdMax;
  this->cmdMin = _cmdMin;

  this->Reset();
}

/////////////////////////////////////////////////
void PID::SetPGain(double _p)
{
  this->pGain = _p;
}

/////////////////////////////////////////////////
void PID::SetIGain(double _i)
{
  this->iGain = _i;
}

/////////////////////////////////////////////////
void PID::SetDGain(double _d)
{
  this->dGain = _d;
}

/////////////////////////////////////////////////
void PID::SetIMax(double _i)
{
  this->iMax = _i;
}

/////////////////////////////////////////////////
void PID::SetIMin(double _i)
{
  this->iMin = _i;
}

/////////////////////////////////////////////////
void PID::SetCmdMax(double _c)
{
  this->cmdMax = _c;
}

/////////////////////////////////////////////////
void PID::SetCmdMin(double _c)
{
  this->cmdMin = _c;
}

/////////////////////////////////////////////////
void PID::Reset()
{
  this->pErrLast = 0.0;
  this->pErr = 0.0;
  this->iErr = 0.0;
  this->dErr = 0.0;
  this->cmd = 0.0;
}

/////////////////////////////////////////////////
double PID::Update(double _error, common::Time _dt)
{
  double pTerm, dTerm, iTerm;
  this->pErr = _error;

  if (_dt == common::Time(0, 0) || math::isnan(_error) || std::isinf(_error))
    return 0.0;

  // Calculate proportional contribution to command
  pTerm = this->pGain * this->pErr;

  // Calculate the integral error
  this->iErr = this->iErr + _dt.Double() * this->pErr;

  // Calculate integral contribution to command
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
  this->cmd = -pTerm - iTerm - dTerm;

  // Check the command limits
  if (!math::equal(this->cmdMax, 0.0) && this->cmd > this->cmdMax)
    this->cmd = this->cmdMax;
  if (!math::equal(this->cmdMin, 0.0) && this->cmd < this->cmdMin)
    this->cmd = this->cmdMin;

  return this->cmd;
}

/////////////////////////////////////////////////
void PID::SetCmd(double _cmd)
{
  this->cmd = _cmd;
}

/////////////////////////////////////////////////
double PID::GetCmd()
{
  return this->cmd;
}

/////////////////////////////////////////////////
void PID::GetErrors(double &_pe, double &_ie, double &_de)
{
  _pe = this->pErr;
  _ie = this->iErr;
  _de = this->dErr;
}
