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

#ifndef __GAZEBO_PID_HH__
#define __GAZEBO_PID_HH__

#include "common/Time.hh"

namespace gazebo
{
  namespace common
  {
    class PID
    {
      ///  \brief Constructor, zeros out Pid values when created and
      ///  initialize Pid-gains and integral term limits:[iMax:iMin]-[I1:I2].
      ///  \param _p  The proportional gain.
      ///  \param _i  The integral gain.
      ///  \param _d  The derivative gain.
      ///  \param _imax The integral upper limit.
      ///  \param _imin The integral lower limit.
      public: PID(double _p = 0.0, double _i = 0.0, double _d = 0.0,
                  double _imax = 0.0, double _imin = 0.0,
                  double _cmdMax = 0.0, double _cmdMin = 0.0);

      /// \brief Destructor
      public: virtual ~PID();

      ///  \brief Initialize PID-gains and integral term
      ///         limits:[iMax:iMin]-[I1:I2]
      ///  \param _p  The proportional gain.
      ///  \param _i  The integral gain.
      ///  \param _d  The derivative gain.
      ///  \param _imax The integral upper limit.
      ///  \param _imin The integral lower limit.
      public: void Init(double _p = 0.0, double _i = 0.0, double _d = 0.0,
                        double _imax = 0.0, double _imin = 0.0,
                        double _cmdMax = 0.0, double _cmdMin = 0.0);

      /// \brief Set the proportional Gain
      /// \param _p proportional gain value
      public: void SetPGain(double _p);

      /// \brief Set the integral Gain
      /// \param _p integral gain value
      public: void SetIGain(double _i);

      /// \brief Set the derivtive Gain
      /// \param _p dertivative gain value
      public: void SetDGain(double _d);

      /// \brief Set the integral upper limit
      /// \param _p integral upper limit value
      public: void SetIMax(double _i);

      /// \brief Set the integral lower limit
      /// \param _p integral lower limit value
      public: void SetIMin(double _i);

      /// \brief Set the maximum value for the command
      /// \param _c The maximum value
      public: void SetCmdMax(double _c);

      /// \brief Set the maximum value for the command
      /// \param _c The maximum value
      public: void SetCmdMin(double _c);

      /// \brief Update the Pid loop with nonuniform time step size.
      /// \param _error  Error since last call (p_state-p_target)
      /// \param _dt Change in time since last call
      public: double Update(double _error, common::Time _dt);

      /// \brief Set current command for this PID controller
      /// \param _cmd New command
      public: void SetCmd(double _cmd);

      /// \brief Return current command for this PID controller
      public: double GetCmd();

      /// \brief Return PID error terms for the controller.
      /// \param _pe  The proportional error.
      /// \param _ie  The integral error.
      /// \param _de  The derivative error.
      public: void GetErrors(double &_pe, double &_ie, double &_de);

      public: PID &operator=(const PID &_p)
              {
                if (this == &_p)
                  return *this;

                this->pGain = _p.pGain;
                this->iGain = _p.iGain;
                this->dGain = _p.dGain;
                this->iMax = _p.iMax;
                this->iMin = _p.iMin;
                this->cmdMax = _p.cmdMax;

                this->Reset();
                return *this;
              }

      /// \brief Reset the errors and command
      public: void Reset();

      private: double pErrLast;
      private: double pErr;
      private: double iErr;
      private: double dErr;
      private: double pGain;
      private: double iGain;
      private: double dGain;
      private: double iMax;
      private: double iMin;
      private: double cmd;
      private: double cmdMax;
      private: double cmdMin;
    };
  }
}
#endif
