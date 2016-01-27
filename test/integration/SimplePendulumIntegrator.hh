/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
/* Desc: Simple pendulum motion integrator
 * Author: John Hsu
 * Date: 2012/04/20
 */

#include <math.h>

// integrate d^2 theta / dt^2 = - g * sin(theta) / l
// by discretizing via central differencing (requires very small dt)
//   (theta_2 - 2*theta_1 + theta_2) / dt^2 = -g * sin(theta_1) / l
//
//   or euler + 3-4-1 backward Euler is 2nd order:
//
//   (2*theta_2 - 5*theta_1 + 4*theta_2 -theta_3) / dt^2
//     = -g * sin(theta_1) / l
//
// where thata_3 is theta(t - 3*dt)
//       thata_2 is theta(t - 2*dt)
//       thata_1 is theta(t -   dt)
//       and
//       thata_f or theta(t_f) is the solution returned
//
// If pendulum starts out stationary, one can assume
//   theta_3 = theta_2 = theta_1 = theta_i
//
double PendulumAngle(double g, double l, double theta_i,
                     double t_i, double t_f , double dt)
{
  double theta_3 = theta_i;
  double theta_2 = theta_i;
  double theta_1 = theta_i;
  double theta_f = theta_i;
  int steps = ceil((t_f - t_i) / dt);
  double t = t_i;
  for (int i = 0 ; i < steps; i++)
  {
    t += dt;
    theta_f = (-dt*dt*g*sin(theta_1)/l
                + theta_3
                - 4.0*theta_2
                + 5.0*theta_1)/2.0;
    /*
    theta_f = (-dt*dt*g*sin(theta_1)/l
                - 1.0*theta_2
                + 2.0*theta_1);
    */
    // next step
    theta_3 = theta_2;
    theta_2 = theta_1;
    theta_1 = theta_f;
    // printf("debug t[%f] t_f[%f] theta_f[%f]\n", t, t_f, theta_f);
  }
  if (fabs(t - t_f) > 0.000001)
    printf("time mismatch t[%f] t_f[%f] theta_f[%f]\n", t, t_f, theta_f);

  return theta_f;
}
