/* Copyright (C)
 *     John Hsu
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
 */
/* Desc: Simple pendulum motion integrator
 * Author: John Hsu
 * Date: 2012/04/20
 */

#include <math.h>

// integrate d^2 theta / dt^2 = - g * sin(theta) / l
// by discretizing it as
//   theta_2^2 - 2*theta_1 + theta_0 / dt^2 = -g * sin(theta_1) / l
//
// where thata_0 is theta(t_1 - 2*dt)
//       thata_1 is theta(t_1)
//       and
//       thata_f or theta(t_f) is the solution returned
//
// If pendulum starts out stationary, one can assume theta_0 = theta_1
//
double PendulumAngle(double g, double l, double theta_0, double theta_1,
                     double t_1, double t_f , double dt)
{
  double theta_f = theta_1;
  int steps = ceil((t_f - t_1) / dt);
  double t = t_1;
  for (int i = 0 ; i < steps; i++)
  {
    t += dt;
    theta_f = -dt*dt*g*sin(theta_1)/l - theta_0 + 2.0*theta_1;
    // next step
    theta_0 = theta_1;
    theta_1 = theta_f;
  }
  if (t != t_f) printf("time mismatch t[%f] t_f[%f] theta_f[%f]\n", t, t_f,
                       theta_f);

  return theta_f;
}
