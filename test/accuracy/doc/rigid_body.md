# Physics Accuracy Testing for a Single Rigid Body

This document will describe simple motion of a rigid body for which analytical solutions can be computed.
  Consider an inertial frame <math>O</math> and a rigid body with coordinate frame <math>c</math>
  attached at the center of mass:

* The position of the center of gravity (cg) in <math>O</math> is given by <math>\textbf{c}</math>.
* The orientation of <math>c</math> with respect to <math>O</math> is given by the quaternion <math>q</math>.

The rigid body has the following inertial parameters:

* mass <math>m</math>
* inertia matrix <math>\textbf{I}</math> expressed in <math>c</math>

Further definitions:

* A rotation matrix from <math>O</math> to <math>q</math>: <math>\textbf{R}(\textbf{q})</math>
* Angular velocity in frame <math>c</math>: <math>\boldsymbol{\omega}</math>.

For time <math>t</math>, denote the initial conditions:

* <math>\textbf{c}(t=t_0) = \textbf{c}_0</math>,
* <math>\dot{\textbf{c}}(t=t_0) = \dot{\textbf{c}}_0</math>,
* <math>\textbf{q}(t=t_0) = \textbf{q}_0</math>,
* <math>\boldsymbol{\omega}(t=t_0) = \boldsymbol{\omega}_0</math>.

Let <math>\textbf{p}(t)</math> represent linear momentum:

* <math>\textbf{p}(t) = m \dot{\textbf{c}}(t)</math>,
* initial value of <math>\textbf{p}_0</math>.

Let <math>\textbf{H}(t)</math> represent the angular momentum with respect to
the center of gravity, expressed in <math>O</math>:

* <math>\textbf{H}(t) = \textbf{R}^T(\textbf{q}(t))\textbf{I}\boldsymbol{\omega}(t)</math>,
* initial value of <math>\textbf{H}_0</math>

Let <math>T(t)</math> represent the kinetic energy as
<math>T(t) = \frac{1}{2} m \dot{\textbf{c}}^T \dot{\textbf{c}} +
     \frac{1}{2} \textbf{I} \boldsymbol{\omega}^T \boldsymbol{\omega}</math>,
and an initial value of <math>T_0</math>

## Gravity-free environment

In a gravity-free environment with no external forces, linear momentum, angular momentum, and energy are conserved.

* Linear momentum conservation:

    * <math>\textbf{p}(t) = \textbf{p}_0</math>
    * <math>\dot{\textbf{c}}(t) = \dot{\textbf{c}}_0</math>
    * <math>\textbf{c}(t) = \textbf{c}_0 + \dot{\textbf{c}}_0(t-t_0)</math>

* Angular momentum conservation:
 
    * <math>\textbf{H}(t) = \textbf{H}_0</math>,
    * <math>\textbf{R}^T(\textbf{q(t)})\textbf{I}\boldsymbol{\omega}(t) =
      \textbf{R}^T(\textbf{q}_0)\textbf{I}\boldsymbol{\omega}_0</math>

 * To see gyroscopic effects, use inertial parameters for 1x4x9 box.

~~~
#!/usr/bin/env python
# Inertial parameters for gyroscopic effects in accuracy test
from numpy import diag

# Use inertial parameters for uniform box with dimensions 1x4x9
m = 1.0
box_x = 1
box_y = 4
box_z = 9
Ixx = m/12 * (box_y^2 + box_z^2)
Iyy = m/12 * (box_z^2 + box_x^2)
Izz = m/12 * (box_x^2 + box_y^2)
I = diag([Ixx, Iyy, Izz])
print I
~~~
~~~
[[ 1.08333333  0.          0.        ]
 [ 0.          0.66666667  0.        ]
 [ 0.          0.          0.41666667]]
~~~

* Energy conservation:

    * <math>T(t) = T_0</math>

## With gravity

With gravity vector <math>\textbf{g}</math> acting:
    * Define potential energy <math>V</math> based on gravity direction.
    * Let <math>V_0</math> be the initial potential energy at <math>\textbf{c}_0</math>.
    * The potential energy is then
      <math>V(t) = V_0 + m \textbf{g} \cdot (\textbf{c}(t) - \textbf{c}_0)</math>
    * The total energy is defined as <math>E = T+V</math> with initial value <math>E_0</math>.

* Linear momentum: Newton implies:
    * <math>\dot{\textbf{p}}(t) = m \textbf{g}</math>,
    * <math>\ddot{\textbf{c}}(t) = \textbf{g}</math>,
    * <math>\dot{\textbf{c}}(t) = \dot{\textbf{c}}_0 + \textbf{g} (t-t_0)</math>,
    * <math>\textbf{c}(t) = \textbf{c}_0 + \dot{\textbf{c}}_0(t-t_0) + \frac{1}{2} \textbf{g} (t-t_0)^2</math>

* Angular momentum: see previous section "Gravity-free environment".

* Energy conservation:
    * <math>E(t) = E_0</math>

## Implementation of accuracy test

* Specify initial conditions and simulate rigid body motion for <math>[t_0,t_f]</math>.
* Take measurements with sampling rate <math>t_s</math> of
  <math>\textbf{c}(t), \dot{\textbf{c}}(t), \textbf{q}(t), \boldsymbol{\omega}(t)</math>.
* Compare to the analytical solutions based on momentum and energy conservation by computing max and RMS error.

