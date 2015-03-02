/*************************************************************************
*                                                                       *
* Open Dynamics Engine, Copyright (C) 2001-2003 Russell L. Smith.       *
* All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
*                                                                       *
* This library is free software; you can redistribute it and/or         *
* modify it under the terms of EITHER:                                  *
*   (1) The GNU Lesser General Public License as published by the Free  *
*       Software Foundation; either version 2.1 of the License, or (at  *
*       your option) any later version. The text of the GNU Lesser      *
*       General Public License is included with this library in the     *
*       file LICENSE.TXT.                                               *
*   (2) The BSD-style license that is included with this library in     *
*       the file LICENSE-BSD.TXT.                                       *
*                                                                       *
* This library is distributed in the hope that it will be useful,       *
* but WITHOUT ANY WARRANTY; without even the implied warranty of        *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
* LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
*                                                                       *
*************************************************************************/
#include <ode/common.h>
#include <ode/odemath.h>
#include <ode/rotation.h>
#include <ode/timer.h>
#include <ode/error.h>
#include <ode/matrix.h>
#include <ode/misc.h>
#include "config.h"
#include "objects.h"
#include "joints/joint.h"
#include "util.h"
#include <sys/time.h>
#include "quickstep_util.h"

using namespace ode;
// multiply block of B matrix (q x 6) with 12 dReal per row with C vektor (q)
void quickstep::Multiply1_12q1 (dReal *A, const dReal *B, const dReal *C, int q)
{
  dIASSERT (q>0 && A && B && C);

  dReal a = 0;
  dReal b = 0;
  dReal c = 0;
  dReal d = 0;
  dReal e = 0;
  dReal f = 0;
  dReal s;

  for(int i=0, k = 0; i<q; k += 12, i++)
  {
    s = C[i]; //C[i] and B[n+k] cannot overlap because its value has been read into a temporary.

    //For the rest of the loop, the only memory dependency (array) is from B[]
    a += B[  k] * s;
    b += B[1+k] * s;
    c += B[2+k] * s;
    d += B[3+k] * s;
    e += B[4+k] * s;
    f += B[5+k] * s;
  }

  A[0] = a;
  A[1] = b;
  A[2] = c;
  A[3] = d;
  A[4] = e;
  A[5] = f;
}

//***************************************************************************
// various common computations involving the matrix J

// compute iMJ = inv(M)*J'
void quickstep::compute_invM_JT (int m, dRealPtr J, dRealMutablePtr iMJ, int *jb,
  dxBody * const *body, dRealPtr invMOI)
{
  dRealMutablePtr iMJ_ptr = iMJ;
  dRealPtr J_ptr = J;
  for (int i=0; i<m; J_ptr += 12, iMJ_ptr += 12, i++) {
    int b1 = jb[i*2];
    int b2 = jb[i*2+1];
    dReal k1 = body[b1]->invMass;
    for (int j=0; j<3; j++) iMJ_ptr[j] = k1*J_ptr[j];
    const dReal *invMOIrow1 = invMOI + 12*b1;
    dMultiply0_331 (iMJ_ptr + 3, invMOIrow1, J_ptr + 3);
    if (b2 >= 0) {
      dReal k2 = body[b2]->invMass;
      for (int j=0; j<3; j++) iMJ_ptr[j+6] = k2*J_ptr[j+6];
      const dReal *invMOIrow2 = invMOI + 12*b2;
      dMultiply0_331 (iMJ_ptr + 9, invMOIrow2, J_ptr + 9);
    }
  }
}

// compute out = inv(M)*J'*in.
void quickstep::multiply_invM_JT (int m, int nb, dRealMutablePtr iMJ, int *jb,
  dRealPtr in, dRealMutablePtr out)
{
  dSetZero (out,6*nb);
  dRealPtr iMJ_ptr = iMJ;
  for (int i=0; i<m; i++) {
    int b1 = jb[i*2];
    int b2 = jb[i*2+1];
    const dReal in_i = in[i];
    dRealMutablePtr out_ptr = out + b1*6;
    for (int j=0; j<6; j++) out_ptr[j] += iMJ_ptr[j] * in_i;
    iMJ_ptr += 6;
    if (b2 >= 0) {
      out_ptr = out + b2*6;
      for (int j=0; j<6; j++) out_ptr[j] += iMJ_ptr[j] * in_i;
    }
    iMJ_ptr += 6;
  }
}

// compute out = J*in.
void quickstep::multiply_J (int m, dRealPtr J, int *jb,
  dRealPtr in, dRealMutablePtr out)
{
  dRealPtr J_ptr = J;
  for (int i=0; i<m; i++) {
    int b1 = jb[i*2];
    int b2 = jb[i*2+1];
    dReal sum = 0;
    dRealPtr in_ptr = in + b1*6;
    for (int j=0; j<6; j++) sum += J_ptr[j] * in_ptr[j];
    J_ptr += 6;
    if (b2 >= 0) {
      in_ptr = in + b2*6;
      for (int j=0; j<6; j++) sum += J_ptr[j] * in_ptr[j];
    }
    J_ptr += 6;
    out[i] = sum;
  }
}

#ifdef USE_CG_LCP
// compute out = (J*inv(M)*J' + cfm)*in.
void quickstep::multiply_J_invM_JT (int m, int nb, dRealMutablePtr J, dRealMutablePtr iMJ, int *jb,
  dRealPtr cfm, dRealMutablePtr z, dRealMutablePtr in, dRealMutablePtr out)
{
  multiply_invM_JT (m,nb,iMJ,jb,in,z);
  multiply_J (m,J,jb,z,out);

  // add cfm
  for (int i=0; i<m; i++) out[i] += cfm[i] * in[i];
}
#endif

#ifdef REORDER_CONSTRAINTS
int quickstep::compare_index_error (const void *a, const void *b)
{
  const IndexError *i1 = (IndexError*) a;
  const IndexError *i2 = (IndexError*) b;
  if (i1->findex < 0 && i2->findex >= 0) return -1;
  if (i1->findex >= 0 && i2->findex < 0) return 1;
  if (i1->error < i2->error) return -1;
  if (i1->error > i2->error) return 1;
  return 0;
}
#endif

// Modifying inertia along constrained axes without modifying dynamics.
void quickstep::DYNAMIC_INERTIA(const int infom, const dxJoint::Info2 &Jinfo, const int b1, const int b2,
                            const dJointWithInfo1 *jicurr,
                            dRealMutablePtr invMOI, dRealMutablePtr MOI)
{
  /// INERTIA PROPAGATION ACROSS CONSTRAINED JOINTS
  for (int j=0; j<infom; j++) {
#ifdef DEBUG_INERTIA_PROPAGATION
    printf("--------JAC---------------\n");
    printf("jacobian [%d] J1l [%f %f %f] J2l [%f %f %f] J1a [%f %f %f]"
           " J2a [%f %f %f]\n", j,
           Jinfo.J1l[0+j*Jinfo.rowskip],
           Jinfo.J1l[1+j*Jinfo.rowskip],
           Jinfo.J1l[2+j*Jinfo.rowskip],
           Jinfo.J2l[0+j*Jinfo.rowskip],
           Jinfo.J2l[1+j*Jinfo.rowskip],
           Jinfo.J2l[2+j*Jinfo.rowskip],
           Jinfo.J1a[0+j*Jinfo.rowskip],
           Jinfo.J1a[1+j*Jinfo.rowskip],
           Jinfo.J1a[2+j*Jinfo.rowskip],
           Jinfo.J2a[0+j*Jinfo.rowskip],
           Jinfo.J2a[1+j*Jinfo.rowskip],
           Jinfo.J2a[2+j*Jinfo.rowskip]);
#endif
    /// \FIXME: For now, implement only for the two non-free axial rotation
    /// constraints for hinge joints.
    /// this only makes sense if joint connects two dynamic bodies (b2 >= 0)
    /// Skip this entire block if we don't want to modify inertia for stability.
    if (b2 >= 0 && jicurr->joint->type() == dJointTypeHinge &&
        (j == 3 || j == 4))
    {
      /// In hinge joint, pure rotational constraint,
      /// J1l and J2l should be zeros, and J1a and J2a should be equal
      /// and opposite to each other.
      /// J1a or J2a indicates the constrained axis direction.
      /// For this implementation, determine constrained axis(s)
      /// direction from J1a for hinge joints.


      /// get the moment of inertia (MOI) for parent and child bodies constrained by J1a and J2a.
      /// MOI and invMOI are already in inertial frame (previously rotated by body.posr.R)
      /// get pointers to our invMOI/MOI matrices
      dReal *invMOI_ptr1 = invMOI + b1 * 12;
      dReal *invMOI_ptr2 = invMOI + b2 * 12;
      dReal *MOI_ptr1 = MOI + b1 * 12;
      dReal *MOI_ptr2 = MOI + b2 * 12;

      // S: unit vector in the constrained axis direction
      // S is the line along which we want to compute MOI
      // FIXME:  check that directions of J1a == J2a
      dVector3 S =
        {Jinfo.J1a[0+j*Jinfo.rowskip],
         Jinfo.J1a[1+j*Jinfo.rowskip],
         Jinfo.J1a[2+j*Jinfo.rowskip] };
      dNormalize3(S);
      // temporary vector used for matrix/vector math
      dVector3 tmp31;

      // compute scalar MOI in line with S for each body
      //   m1 = S' * MOI1 * S
      dMultiply0_133(tmp31, S, MOI_ptr1);
      dReal m1 = dCalcVectorDot3(tmp31, S);

      //   m2 = S' * MOI2 * S
      dMultiply0_133(tmp31, S, MOI_ptr2);
      dReal m2 = dCalcVectorDot3(tmp31, S);

      // identify body with larger inertia
      dReal m_large = m1;
      dReal m_small = m2;
      dReal *MOI_large = MOI_ptr1;
      dReal *MOI_small = MOI_ptr2;
      if (m2 > m1)
      {
        m_large = m2;
        m_small = m1;
        MOI_large = MOI_ptr2;
        MOI_small = MOI_ptr1;
      }

      /// get full axis MOI tensor representing the scalar axis MOI.
      // full MOI tensor for S needs matrix outer product of S:
      //   SS = [ S * S' ]
      dMatrix3 SS = {
           S[0]*S[0], S[0]*S[1], S[0]*S[2], 0,
           S[1]*S[0], S[1]*S[1], S[1]*S[2], 0,
           S[2]*S[0], S[2]*S[1], S[2]*S[2], 0};

#ifdef DEBUG_INERTIA_PROPAGATION
      printf("--------old MOI-----------\n");
      printf("MOI1[%d]\n[%f %f %f %f]\n[%f %f %f %f]\n[%f %f %f %f]\n", b1,
        MOI_ptr1[0*4+0],MOI_ptr1[0*4+1],MOI_ptr1[0*4+2],MOI_ptr1[0*4+3],
        MOI_ptr1[1*4+0],MOI_ptr1[1*4+1],MOI_ptr1[1*4+2],MOI_ptr1[1*4+3],
        MOI_ptr1[2*4+0],MOI_ptr1[2*4+1],MOI_ptr1[2*4+2],MOI_ptr1[2*4+3]);
      printf("MOI2[%d]\n[%f %f %f %f]\n[%f %f %f %f]\n[%f %f %f %f]\n", b2,
        MOI_ptr2[0*4+0],MOI_ptr2[0*4+1],MOI_ptr2[0*4+2],MOI_ptr2[0*4+3],
        MOI_ptr2[1*4+0],MOI_ptr2[1*4+1],MOI_ptr2[1*4+2],MOI_ptr2[1*4+3],
        MOI_ptr2[2*4+0],MOI_ptr2[2*4+1],MOI_ptr2[2*4+2],MOI_ptr2[2*4+3]);
      printf("--------S VECTORS-----------\n");
      printf("MOI1 b1[%d] S[%f %f %f] = %g\n",b1, S[0], S[1], S[2], m1);
      printf("MOI2 b2[%d] S[%f %f %f] = %g\n",b2, S[0], S[1], S[2], m2);
      printf("--------SS----------------\n");
      printf("SS [%d]\n[%f %f %f %f]\n[%f %f %f %f]\n[%f %f %f %f]\n", b1,
        SS[0*4+0],SS[0*4+1],SS[0*4+2],SS[0*4+3],
        SS[1*4+0],SS[1*4+1],SS[1*4+2],SS[1*4+3],
        SS[2*4+0],SS[2*4+1],SS[2*4+2],SS[2*4+3]);
#endif

      // define maximum ratio of moment of inertia for adjacent bodies
      // ie. m_large / m_small <= moi_ratio_max
      /// \todo make moi_ratio_max adjustable
      /// \todo automatically adjust moi_ratio_max such that
      /// abs sum of off-diagonals remains smaller than the diagonal
      /// for all rows (see comments below about Gauss-Seidel stability).
      // increase moi_ratio_max to skip checks and increase performance
      const dReal moi_ratio_max = 200.0;
      if (m_large > moi_ratio_max * m_small)
      {
        // Large inertia ratio detected, try reducing it.
        // Increase m_small by dm
        //   Reduce m_large by dm
        //   such that (m_large - dm) = moi_ratio_max * (m_small + dm)
        // Intermediate math step:
        //   m_large - moi_ratio_max * m_small = dm * (1 + moi_ratio_max)
        // Then:
        dReal dm = (m_large - moi_ratio_max * m_small) / (1 + moi_ratio_max);

        // This will then be applied to the bodies by multiplying by [S*S']
        //   MOI_large -= dm * SS
        //   MOI_small += dm * SS
        // But first it should be verified that the change will not
        // destabilize the Gauss-Seidel solver.
        // To verify this, the Generalized Line Criterion for Gauss-Seidel
        // is used, which is stated below:
        //
        // The Gauss-Seidel method will converge if for each row of the matrix:
        //   the sum of absolute values of off-diagonal elements
        //   is less than absolute value of the diagonal element.
        //
        // @article{Garcia2003,
        // author = {Garcia, M.V.P. and {Humes Jr.}, C. and Stern, J.M.},
        // doi = {10.1590/S0101-82052003000100006},
        // issn = {0101-8205},
        // journal = {Computational \& Applied Mathematics},
        // number = {1},
        // pages = {91--97},
        // title = {{Generalized line criterion for Gauss-Seidel method}},
        // url = {http://www.ime.usp.br/~jstern/papers/papersJS/ghs03.pdf},
        // volume = {22},
        // year = {2003}
        // }

        // For 3x3 Gauss-Seidel matrix M, this is equivalent to:
        //   abs(M[0][0]) > abs(M[0][1]) + abs(M[0][2])
        //   abs(M[1][1]) > abs(M[1][0]) + abs(M[1][2])
        //   abs(M[2][2]) > abs(M[2][0]) + abs(M[2][1])
        //
        // Since M is a mass matrix, it is positive definite, which implies
        // that the diagonal elements are strictly positive:
        //   M[0][0] > abs(M[0][1]) + abs(M[0][2])
        //   M[1][1] > abs(M[1][0]) + abs(M[1][2])
        //   M[2][2] > abs(M[2][0]) + abs(M[2][1])
        //
        // For extra safety factor define a parameter gamma >= 1 such that:
        //   M[0][0] > gamma * (abs(M[0][1]) + abs(M[0][2]))
        //   M[1][1] > gamma * (abs(M[1][0]) + abs(M[1][2]))
        //   M[2][2] > gamma * (abs(M[2][0]) + abs(M[2][1]))
        const dReal gamma = 1.0;

        int problem = 0;
        for (int row = 0; row < 3; ++row)
        {
          // off-diagonal columns
          int col1 = (row + 1) % 3;
          int col2 = (row + 2) % 3;

          // diagonal index
          int id = row*4 + row;

          // off-diagonal indices
          int iod1 = row*4 + col1;
          int iod2 = row*4 + col2;

          // diagonal element of adjusted M_large
          dReal Md_large = MOI_large[id] - dm*SS[id];

          // sum of absolute values of off-diagonal elements of adjusted M_large
          dReal Mod_large = fabs(MOI_large[iod1] - dm*SS[iod1])
                          + fabs(MOI_large[iod2] - dm*SS[iod2]);

          // diagonal element of adjusted M_small
          dReal Md_small = MOI_small[id] + dm*SS[id];

          // sum of absolute values of off-diagonal elements of adjusted M_small
          dReal Mod_small = fabs(MOI_small[iod1] + dm*SS[iod1])
                          + fabs(MOI_small[iod2] + dm*SS[iod2]);

          if (Md_large <= gamma*Mod_large  ||  Md_small <= gamma*Mod_small)
          {
            problem = 1;
          }
        }

        if (problem == 0)
        {
          // Everything looks good, update the inertia matrices
          for (int i = 0; i < 12; ++i)
          {
            int col = i%4;
            if (col == 3)
            {
              //  set unused terms to zero
              MOI_large[i] = 0;
              MOI_small[i] = 0;
            }
            else
            {
              MOI_large[i] -= dm * SS[i];
              MOI_small[i] += dm * SS[i];
            }
          }
        }

          // Update invMOI by inverting analytically (may not be efficient).
          // try 1981 Ken Miller (http://www.jstor.org/stable/2690437) or
          //   (http://math.stackexchange.com/questions/17776)
          // try taking advantage of symmetry of MOI
          dReal det1 = MOI_ptr1[0*4+0]*(MOI_ptr1[2*4+2]*MOI_ptr1[1*4+1]
                      -MOI_ptr1[2*4+1]*MOI_ptr1[1*4+2])
                      -MOI_ptr1[1*4+0]*(MOI_ptr1[2*4+2]*MOI_ptr1[0*4+1]
                      -MOI_ptr1[2*4+1]*MOI_ptr1[0*4+2])
                      +MOI_ptr1[2*4+0]*(MOI_ptr1[1*4+2]*MOI_ptr1[0*4+1]
                      -MOI_ptr1[1*4+1]*MOI_ptr1[0*4+2]);
          invMOI_ptr1[0*4+0] =  (MOI_ptr1[2*4+2]*MOI_ptr1[1*4+1]
                                -MOI_ptr1[2*4+1]*MOI_ptr1[1*4+2])/det1;
          invMOI_ptr1[0*4+1] = -(MOI_ptr1[2*4+2]*MOI_ptr1[0*4+1]
                                -MOI_ptr1[2*4+1]*MOI_ptr1[0*4+2])/det1;
          invMOI_ptr1[0*4+2] =  (MOI_ptr1[1*4+2]*MOI_ptr1[0*4+1]
                                -MOI_ptr1[1*4+1]*MOI_ptr1[0*4+2])/det1;
          // invMOI_ptr1[0*4+3] = 0.0;
          invMOI_ptr1[1*4+0] = invMOI_ptr1[0*4+1];
          invMOI_ptr1[1*4+1] =  (MOI_ptr1[2*4+2]*MOI_ptr1[0*4+0]
                                -MOI_ptr1[2*4+0]*MOI_ptr1[0*4+2])/det1;
          invMOI_ptr1[1*4+2] = -(MOI_ptr1[1*4+2]*MOI_ptr1[0*4+0]
                                -MOI_ptr1[1*4+0]*MOI_ptr1[0*4+2])/det1;
          // invMOI_ptr1[1*4+3] = 0.0;
          invMOI_ptr1[2*4+0] = invMOI_ptr1[0*4+2];
          invMOI_ptr1[2*4+1] = invMOI_ptr1[1*4+2];
          invMOI_ptr1[2*4+2] =  (MOI_ptr1[1*4+1]*MOI_ptr1[0*4+0]
                                -MOI_ptr1[1*4+0]*MOI_ptr1[0*4+1])/det1;
          // invMOI_ptr1[2*4+3] = 0.0;

          dReal det2 = MOI_ptr2[0*4+0]*(MOI_ptr2[2*4+2]*MOI_ptr2[1*4+1]
                      -MOI_ptr2[2*4+1]*MOI_ptr2[1*4+2])
                      -MOI_ptr2[1*4+0]*(MOI_ptr2[2*4+2]*MOI_ptr2[0*4+1]
                      -MOI_ptr2[2*4+1]*MOI_ptr2[0*4+2])
                      +MOI_ptr2[2*4+0]*(MOI_ptr2[1*4+2]*MOI_ptr2[0*4+1]
                      -MOI_ptr2[1*4+1]*MOI_ptr2[0*4+2]);
          invMOI_ptr2[0*4+0] =  (MOI_ptr2[2*4+2]*MOI_ptr2[1*4+1]
                                -MOI_ptr2[2*4+1]*MOI_ptr2[1*4+2])/det2;
          invMOI_ptr2[0*4+1] = -(MOI_ptr2[2*4+2]*MOI_ptr2[0*4+1]
                                -MOI_ptr2[2*4+1]*MOI_ptr2[0*4+2])/det2;
          invMOI_ptr2[0*4+2] =  (MOI_ptr2[1*4+2]*MOI_ptr2[0*4+1]
                                -MOI_ptr2[1*4+1]*MOI_ptr2[0*4+2])/det2;
          // invMOI_ptr2[0*4+3] = 0.0;
          invMOI_ptr2[1*4+0] = invMOI_ptr2[0*4+1];
          invMOI_ptr2[1*4+1] =  (MOI_ptr2[2*4+2]*MOI_ptr2[0*4+0]
                                -MOI_ptr2[2*4+0]*MOI_ptr2[0*4+2])/det2;
          invMOI_ptr2[1*4+2] = -(MOI_ptr2[1*4+2]*MOI_ptr2[0*4+0]
                                -MOI_ptr2[1*4+0]*MOI_ptr2[0*4+2])/det2;
          // invMOI_ptr2[1*4+3] = 0.0;
          invMOI_ptr2[2*4+0] = invMOI_ptr2[0*4+2];
          invMOI_ptr2[2*4+1] = invMOI_ptr2[1*4+2];
          invMOI_ptr2[2*4+2] =  (MOI_ptr2[1*4+1]*MOI_ptr2[0*4+0]
                                -MOI_ptr2[1*4+0]*MOI_ptr2[0*4+1])/det2;
          // invMOI_ptr2[2*4+3] = 0.0;

  #ifdef DEBUG_INERTIA_PROPAGATION
        printf("---------S Scalars--------\n");
        printf(" original    S1 [%g] S2 [%g]\n", m1, m2);
        printf(" distributed S1 [%g] S2 [%g]\n", m1_new, m2_new);
          printf("----------new MOI---------\n");

          printf("new MOI1[%d]\n[%f %f %f %f]\n[%f %f %f %f]\n[%f %f %f %f]\n",
            b1,
            MOI_ptr1[0*4+0],MOI_ptr1[0*4+1],MOI_ptr1[0*4+2],MOI_ptr1[0*4+3],
            MOI_ptr1[1*4+0],MOI_ptr1[1*4+1],MOI_ptr1[1*4+2],MOI_ptr1[1*4+3],
            MOI_ptr1[2*4+0],MOI_ptr1[2*4+1],MOI_ptr1[2*4+2],MOI_ptr1[2*4+3]);

          // Modify MOI_ptr2
          printf("new MOI2[%d]\n[%f %f %f %f]\n[%f %f %f %f]\n[%f %f %f %f]\n",
            b2,
            MOI_ptr2[0*4+0],MOI_ptr2[0*4+1],MOI_ptr2[0*4+2],MOI_ptr2[0*4+3],
            MOI_ptr2[1*4+0],MOI_ptr2[1*4+1],MOI_ptr2[1*4+2],MOI_ptr2[1*4+3],
            MOI_ptr2[2*4+0],MOI_ptr2[2*4+1],MOI_ptr2[2*4+2],MOI_ptr2[2*4+3]);

          // double check resulting MOI along s
          dMultiply0_133(tmp31, S, MOI_ptr1);
          // scalar body 1 MOI component along vector S
          m1 = dCalcVectorDot3(tmp31, S);
          printf("new MOI1 along S [%f]\n", m1);
          dMultiply0_133(tmp31, S, MOI_ptr2);
          // scalar body 2 MOI component along vector S
          m2 = dCalcVectorDot3(tmp31, S);
          printf("new MOI2 along S [%f]\n", m2);

          /// \todo double check resulting MOI along joint axis and
          /// see that it's the same

          printf("----------new inv---------\n");
          printf("new invMOI1[%d]\n[%f %f %f %f]\n[%f %f %f %f]\n"
                 "[%f %f %f %f]\n", b1,
                 invMOI_ptr1[0*4+0], invMOI_ptr1[0*4+1],
                 invMOI_ptr1[0*4+2], invMOI_ptr1[0*4+3],
                 invMOI_ptr1[1*4+0], invMOI_ptr1[1*4+1],
                 invMOI_ptr1[1*4+2], invMOI_ptr1[1*4+3],
                 invMOI_ptr1[2*4+0], invMOI_ptr1[2*4+1],
                 invMOI_ptr1[2*4+2], invMOI_ptr1[2*4+3]);

          // Modify invMOI_ptr2
          printf("new invMOI2[%d]\n[%f %f %f %f]\n[%f %f %f %f]\n"
                 "[%f %f %f %f]\n", b2,
                 invMOI_ptr2[0*4+0], invMOI_ptr2[0*4+1],
                 invMOI_ptr2[0*4+2], invMOI_ptr2[0*4+3],
                 invMOI_ptr2[1*4+0], invMOI_ptr2[1*4+1],
                 invMOI_ptr2[1*4+2], invMOI_ptr2[1*4+3],
                 invMOI_ptr2[2*4+0], invMOI_ptr2[2*4+1],
                 invMOI_ptr2[2*4+2], invMOI_ptr2[2*4+3]);
  #endif

  #ifdef DEBUG_INERTIA_PROPAGATION
          // check if diagonally-dominant
          if (MOI_ptr1[0*4+0] < dFabs(MOI_ptr1[0*4+1])+dFabs(MOI_ptr1[0*4+2]))
            printf(" * new MOI1 row 1 d[%f] < o[%f, %f]\n",
                   MOI_ptr1[0*4+0],MOI_ptr1[0*4+1], MOI_ptr1[0*4+2]);
          if (MOI_ptr1[1*4+1] < dFabs(MOI_ptr1[1*4+0])+dFabs(MOI_ptr1[1*4+2]))
            printf(" * new MOI1 row 2 d[%f] < o[%f, %f]\n",
                   MOI_ptr1[1*4+1],MOI_ptr1[1*4+0], MOI_ptr1[1*4+2]);
          if (MOI_ptr1[2*4+2] < dFabs(MOI_ptr1[2*4+0])+dFabs(MOI_ptr1[2*4+1]))
            printf(" * new MOI1 row 3 d[%f] < o[%f, %f]\n",
                   MOI_ptr1[2*4+2],MOI_ptr1[2*4+0], MOI_ptr1[2*4+1]);

          if (MOI_ptr2[0*4+0] < dFabs(MOI_ptr2[0*4+1])+dFabs(MOI_ptr2[0*4+2]))
            printf(" * new MOI2 row 1 d[%f] < o[%f, %f]\n",
                   MOI_ptr2[0*4+0],MOI_ptr2[0*4+1], MOI_ptr2[0*4+2]);
          if (MOI_ptr2[1*4+1] < dFabs(MOI_ptr2[1*4+0])+dFabs(MOI_ptr2[1*4+2]))
            printf(" * new MOI2 row 2 d[%f] < o[%f, %f]\n",
                   MOI_ptr2[1*4+1],MOI_ptr2[1*4+0], MOI_ptr2[1*4+2]);
          if (MOI_ptr2[2*4+2] < dFabs(MOI_ptr2[2*4+0])+dFabs(MOI_ptr2[2*4+1]))
            printf(" * new MOI2 row 3 d[%f] < o[%f, %f]\n",
                   MOI_ptr2[2*4+2],MOI_ptr2[2*4+0], MOI_ptr2[2*4+1]);
  #endif
      }
    }
  }
}
