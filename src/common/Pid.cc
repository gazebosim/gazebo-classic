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

// Original version: Melonee Wise <mwise@willowgarage.com>

#include <tinyxml.h>
#include "common/Pid.hh"
#include "math/gzmath.h"

using namespace gazebo;
using namespace common;

Pid::Pid(double P, double I, double D, double I1, double I2)
  : p_gain_(P), i_gain_(I), d_gain_(D), i_max_(I1), i_min_(I2)
{
  p_error_last_ = 0.0;
  p_error_ = 0.0;
  d_error_ = 0.0;
  i_error_ = 0.0;
  cmd_ = 0.0;
}

Pid::~Pid()
{
}

void Pid::initPid(double P, double I, double D, double I1, double I2)
{
  p_gain_ = P;
  i_gain_ = I;
  d_gain_ = D;
  i_max_ = I1;
  i_min_ = I2;

  reset();
}

void Pid::reset()
{
  p_error_last_ = 0.0;
  p_error_ = 0.0;
  d_error_ = 0.0;
  i_error_ = 0.0;
  cmd_ = 0.0;
}

void Pid::getGains(double &p, double &i, double &d, double &i_max,
                   double &i_min)
{
  p = p_gain_;
  i = i_gain_;
  d = d_gain_;
  i_max = i_max_;
  i_min = i_min_;
}

void Pid::setGains(double P, double I, double D, double I1, double I2)
{
  p_gain_ = P;
  i_gain_ = I;
  d_gain_ = D;
  i_max_ = I1;
  i_min_ = I2;
}

bool Pid::initXml(TiXmlElement *config)
{
  p_gain_ = config->Attribute("p") ? atof(config->Attribute("p")) : 0.0;
  i_gain_ = config->Attribute("i") ? atof(config->Attribute("i")) : 0.0;
  d_gain_ = config->Attribute("d") ? atof(config->Attribute("d")) : 0.0;
  i_max_ = config->Attribute("iClamp") ?
           atof(config->Attribute("iClamp")) : 0.0;
  i_min_ = -i_max_;

  reset();
  return true;
}

double Pid::updatePid(double error, Time dt)
{
  double p_term, d_term, i_term;
  p_error_ = error;  // this is pError = pState-pTarget

  if (dt == Time(0.0) || isnan(error) || isinf(error))
    return 0.0;

  // Calculate proportional contribution to command
  p_term = p_gain_ * p_error_;

  // Calculate the integral error
  i_error_ = i_error_ + dt.Double() * p_error_;

  // Calculate integral contribution to command
  i_term = i_gain_ * i_error_;

  // Limit i_term so that the limit is meaningful in the output
  if (i_term > i_max_)
  {
    i_term = i_max_;
    i_error_ = i_term/i_gain_;
  }
  else if (i_term < i_min_)
  {
    i_term = i_min_;
    i_error_ = i_term/i_gain_;
  }

  // Calculate the derivative error
  if (dt.Double() != 0)
  {
    d_error_ = (p_error_ - p_error_last_) / dt.Double();
    p_error_last_ = p_error_;
  }
  // Calculate derivative contribution to command
  d_term = d_gain_ * d_error_;
  cmd_ = -p_term - i_term - d_term;

  return cmd_;
}


double Pid::updatePid(double error, double error_dot, Time dt)
{
  double p_term, d_term, i_term;
  p_error_ = error;  // this is pError = pState-pTarget
  d_error_ = error_dot;

  if (dt == Time(0.0) || isnan(error) || isinf(error)
                      || isnan(error_dot) || isinf(error_dot))
    return 0.0;


  // Calculate proportional contribution to command
  p_term = p_gain_ * p_error_;

  // Calculate the integral error
  i_error_ = i_error_ + dt.Double() * p_error_;

  // Calculate integral contribution to command
  i_term = i_gain_ * i_error_;

  // Limit i_term so that the limit is meaningful in the output
  if (i_term > i_max_)
  {
    i_term = i_max_;
    i_error_ = i_term/i_gain_;
  }
  else if (i_term < i_min_)
  {
    i_term = i_min_;
    i_error_ = i_term/i_gain_;
  }

  // Calculate derivative contribution to command
  d_term = d_gain_ * d_error_;
  cmd_ = -p_term - i_term - d_term;

  return cmd_;
}



void Pid::setCurrentCmd(double cmd)
{
  cmd_ = cmd;
}

double Pid::getCurrentCmd()
{
  return cmd_;
}

void Pid::getCurrentPIDErrors(double *pe, double *ie, double *de)
{
  *pe = p_error_;
  *ie = i_error_;
  *de = d_error_;
}


