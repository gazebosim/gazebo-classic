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
#ifndef CONTROL_TOOLBOX_PID_H
#define CONTROL_TOOLBOX_PID_H


#include <string>
#include "common/Time.hh"

class TiXmlElement;

namespace gazebo {
namespace common {

/***************************************************/
/*! \class Pid
    \brief A basic pid class.

    This class implements a generic structure that
    can be used to create a wide range of pid
    controllers. It can function independently or
    be subclassed to provide more specific controls
    based on a particular control loop.

    In particular, this class implements the standard
    pid equation:

    \f$command  = -p_{term} - i_{term} - d_{term} \f$

    where: <br>
    <UL TYPE="none">
    <LI>  \f$ p_{term}  = p_{gain} * p_{error} \f$
    <LI>  \f$ i_{term}  = i_{gain} * i_{error} \f$
    <LI>  \f$ d_{term}  = d_{gain} * d_{error} \f$
    <LI>  \f$ i_{error} = i_{error} + p_{error} * dt \f$
    <LI>  \f$ d_{error} = (p_{error} - p_{error last}) / dt \f$
    </UL>

    given:<br>
    <UL TYPE="none">
    <LI>  \f$ p_{error}  = p_{state} - p_{target} \f$.
    </UL>

    @param p Proportional gain

    @param d Derivative gain

    @param i Integral gain

    @param i_clamp Min/max bounds for the integral windup, the clamp is applied to the \f$i_{term}\f$ and not the \f$i_{error}\f$

    @section Usage

    To use the Pid class, you should first call some version of init()
    (in non-realtime) and then call updatePid() at every update step.
    For example:

\verbatim
Pid pid;
pid.initPid(6.0, 1.0, 2.0, 0.3, -0.3);
double position_desi_ = 0.5;
...
Time last_time = this->world->GetSimTime();
while (true) {
  Time time = this->world->GetSimTime();
  double effort = pid.updatePid(currentPosition() - position_desi_, time - last_time);
  last_time = time;
}
\endverbatim

*/
/***************************************************/

class Pid
{
  public:

    /*!
     * \brief Constructor, zeros out Pid values when created and
     * initialize Pid-gains and integral term limits:[iMax:iMin]-[I1:I2].
     *
     * \param P  The proportional gain.
     * \param I  The integral gain.
     * \param D  The derivative gain.
     * \param I1 The integral upper limit.
     * \param I2 The integral lower limit.
     */
    Pid(double P = 0.0, double I = 0.0, double D = 0.0, double I1 = 0.0,
        double I2 = -0.0);

    /*!
     * \brief Destructor of Pid class.
     */
    ~Pid();

    /*!
     * \brief Update the Pid loop with nonuniform time step size.
     *
     * \param p_error  Error since last call (p_state-p_target)
     * \param dt Change in time since last call
     */
    double updatePid(double p_error, Time dt);

    /*!
     * \brief Initialize PID-gains and integral term limits:[iMax:iMin]-[I1:I2]
     *
     * \param P  The proportional gain.
     * \param I  The integral gain.
     * \param D  The derivative gain.
     * \param I1 The integral upper limit.
     * \param I2 The integral lower limit.
     */
    void initPid(double P, double I, double D, double I1, double I2);

    /*!                                                                                                   
     * \brief Initialize PID with the parameters in a namespace                               
     *                                                            
     * \param n  The namespace prefix.
     */
    bool initXml(TiXmlElement *config);

    /*!
     * \brief Reset the state of this PID controller
     */
    void reset();

    /*!
     * \brief Set current command for this PID controller
     */
    void setCurrentCmd(double cmd);

    /*!
     * \brief Return current command for this PID controller
     */
    double getCurrentCmd();

    /*!
     * \brief Return PID error terms for the controller.
     * \param pe  The proportional error.
     * \param ie  The integral error.
     * \param de  The derivative error.
     */
    void getCurrentPIDErrors(double *pe, double *ie, double *de);

    /*!
     * \brief Set PID gains for the controller.
     * \param P  The proportional gain.
     * \param I  The integral gain.
     * \param D  The derivative gain.
     * \param i_max
     * \param i_min
     */
    void setGains(double P, double I, double D, double i_max, double i_min);

    /*!
     * \brief Get PID gains for the controller.
     * \param p  The proportional gain.
     * \param i  The integral gain.
     * \param d  The derivative gain.
     * \param i_max The max integral windup.
     * \param i_mim The min integral windup.
     */
    void getGains(double &p, double &i, double &d, double &i_max,
                  double &i_min);

    /*!
     * \brief Update the Pid loop with nonuniform time step size. This update call 
     * allows the user to pass in a precomputed derivative error. 
     *
     * \param error  Error since last call (p_state-p_target)
     * \param error_dot d(Error)/dt since last call
     * \param dt Change in time since last call
     */
    double updatePid(double error, double error_dot, Time dt);

    Pid &operator =(const Pid& p)
    {
      if (this == &p)
        return *this;

      p_gain_ = p.p_gain_;
      i_gain_ = p.i_gain_;
      d_gain_ = p.d_gain_;
      i_max_ = p.i_max_;
      i_min_ = p.i_min_;

      p_error_last_ = p_error_ = i_error_ = d_error_ = cmd_ = 0.0;
      return *this;
    }

  private:
    double p_error_last_; /**< used for derivative state calculation. */
    double p_error_; /**< Position error. */
    double d_error_; /**< Derivative error. */
    double i_error_; /**< Integator error. */
    double p_gain_;  /**< Proportional gain. */
    double i_gain_;  /**< Integral gain. */
    double d_gain_;  /**< Derivative gain. */
    double i_max_;   /**< Maximum allowable integral term. */
    double i_min_;   /**< Minimum allowable integral term. */
    double cmd_;     /**< Command to send. */
};
}
}
#endif
