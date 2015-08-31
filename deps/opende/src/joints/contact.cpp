/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
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


#include "config.h"
#include "contact.h"
#include "joint_internal.h"



//****************************************************************************
// contact

dxJointContact::dxJointContact( dxWorld *w ) :
        dxJoint( w )
{
}


void
dxJointContact::getSureMaxInfo( SureMaxInfo* info )
{
    info->max_m = 4; // ...as the actual m is very likely to hit the maximum
}


void
dxJointContact::getInfo1( dxJoint::Info1 *info )
{
    // make sure mu's >= 0, then calculate number of constraint rows and number
    // of unbounded rows.
    int m = 1, nub = 0;
    if ( contact.surface.mu < 0 ) contact.surface.mu = 0;
    if ( contact.surface.mode & dContactMu2 )
    {
        m += 2;
        if ( contact.surface.mu2 < 0 ) contact.surface.mu2 = 0;
        if (_dequal(contact.surface.mu, dInfinity)) nub ++;
        if (_dequal(contact.surface.mu2, dInfinity)) nub ++;
    }
    else
    {
        if ( contact.surface.mu > 0 ) m += 2;
        if (_dequal(contact.surface.mu, dInfinity)) nub += 2;
    }

    if ( contact.surface.mode & dContactMu3 )
    {
        if ( contact.surface.mu3 < 0 ) contact.surface.mu3 = 0;
        m++;  // do this even if mu3 is zero.
        if (_dequal(contact.surface.mu3, dInfinity)) nub ++;
    }

    the_m = m;
    info->m = m;
    info->nub = nub;
}


void
dxJointContact::getInfo2( dxJoint::Info2 *info )
{
    int s = info->rowskip;
    int s2 = 2 * s;
    int s3 = 3 * s;

    // get normal, with sign adjusted for body1/body2 polarity
    dVector3 normal;
    if ( flags & dJOINT_REVERSE )
    {
        normal[0] = - contact.geom.normal[0];
        normal[1] = - contact.geom.normal[1];
        normal[2] = - contact.geom.normal[2];
    }
    else
    {
        normal[0] = contact.geom.normal[0];
        normal[1] = contact.geom.normal[1];
        normal[2] = contact.geom.normal[2];
    }
    normal[3] = 0; // @@@ hmmm

    // c1,c2 = contact points with respect to body PORs
    dVector3 c1, c2 = {0,0,0};
    c1[0] = contact.geom.pos[0] - node[0].body->posr.pos[0];
    c1[1] = contact.geom.pos[1] - node[0].body->posr.pos[1];
    c1[2] = contact.geom.pos[2] - node[0].body->posr.pos[2];

    // set jacobian for normal
    info->J1l[0] = normal[0];
    info->J1l[1] = normal[1];
    info->J1l[2] = normal[2];
    dCalcVectorCross3( info->J1a, c1, normal );
    if ( node[1].body )
    {
        c2[0] = contact.geom.pos[0] - node[1].body->posr.pos[0];
        c2[1] = contact.geom.pos[1] - node[1].body->posr.pos[1];
        c2[2] = contact.geom.pos[2] - node[1].body->posr.pos[2];
        info->J2l[0] = -normal[0];
        info->J2l[1] = -normal[1];
        info->J2l[2] = -normal[2];
        dCalcVectorCross3( info->J2a, c2, normal );
        dNegateVector3( info->J2a );
    }

    // experimental - check relative acceleration at the contact

    dReal depth;
    dReal min_min_depth;
    if (node[0].body->contactp != NULL && (node[1].body && node[1].body->contactp != NULL))
    {
      min_min_depth = std::min(node[0].body->contactp->min_depth,node[1].body->contactp->min_depth);
      depth = contact.geom.depth - min_min_depth;
    }
    else if (node[0].body->contactp != NULL)
    {
      min_min_depth = node[0].body->contactp->min_depth;
      depth = contact.geom.depth - min_min_depth;
    }
    else if (node[1].body && node[1].body->contactp != NULL)
    {
      min_min_depth = node[1].body->contactp->min_depth;
      depth = contact.geom.depth - min_min_depth;
    }
    else
    {
      min_min_depth = world->contactp.min_depth;
      depth = contact.geom.depth - min_min_depth;
    }
    if ( depth < 0 ) depth = 0;

    if ( contact.surface.mode & dContactSoftCFM )
        info->cfm[0] = contact.surface.soft_cfm;

    dReal motionN = 0;
    if ( contact.surface.mode & dContactMotionN )
        motionN = contact.surface.motionN;

    // set right hand side and cfm value for normal
    dReal local_erp = info->erp;
    if ( contact.surface.mode & dContactSoftERP )
        local_erp = contact.surface.soft_erp;

    if ( contact.surface.mode & dContactEM )
    {
        // get patch radius for surface area calculation
        dReal patch_radius;
        if (!contact.surface.use_patch_radius)
        {
          patch_radius = sqrt(contact.surface.surface_radius * depth);
        }
        else
        {
          patch_radius = contact.surface.patch_radius;
        }

        // use elastic modulus
        dReal e_star = contact.surface.elastic_modulus;
        /// \TODO Using Hertzian contact, but ignoring the ^1.5 power!!!
        /// We should fix this by either linearizing about x, or try
        /// to rederive the cfm/erp -> kp/kd equivalence (per Catto)
        /// for stiffness term = K*x^1.5.
        /// For now, pretend it's just x.
        ///   equation 5.23 form Contact Mechanics and Friction by Popov
        dReal stiffness = 4.0 / 3.0 * e_star * sqrt(patch_radius);

        // convert stiffness to erp (known cfm, h, kp)
        // get kd using:
        //   cfm = 1 / ( dt * kp + kd )
        dReal kd = 1.0/info->cfm[0] - stiffness/info->fps;
        // get erp using:
        //   kd = (1 - erp) / cfm
        local_erp = 1.0 - kd * info->cfm[0];
    }
    dReal k = info->fps * local_erp;

    const dReal pushout = k * depth + motionN;
    info->c[0] = pushout;

    // note: this cap should not limit bounce velocity
    // if contactp is not specified per body, use the global max_vel specified in world
    // otherwise, use the body max_vel, but truncated by world max_vel.
    dReal maxvel = world->contactp.max_vel;
    if (node[0].body->contactp != NULL && (node[1].body && node[1].body->contactp != NULL))
      maxvel = std::min(node[0].body->contactp->max_vel,node[1].body->contactp->max_vel);
    else if (node[0].body && node[0].body->contactp != NULL)
      maxvel = node[0].body->contactp->max_vel;
    else if (node[1].body && node[1].body->contactp != NULL)
      maxvel = node[1].body->contactp->max_vel;

    // truncate everything by world max_vel
    if (maxvel > world->contactp.max_vel)
      maxvel = world->contactp.max_vel;

    info->c_v_max[0] = maxvel;

    // deal with bounce
    if ( contact.surface.mode & dContactBounce )
    {
        // calculate outgoing velocity (-ve for incoming contact)
        dReal outgoing = dCalcVectorDot3( info->J1l, node[0].body->lvel )
                         + dCalcVectorDot3( info->J1a, node[0].body->avel );
        if ( node[1].body )
        {
            outgoing += dCalcVectorDot3( info->J2l, node[1].body->lvel )
                        + dCalcVectorDot3( info->J2a, node[1].body->avel );
        }
        outgoing -= motionN;
        // only apply bounce if the outgoing velocity is greater than the
        // threshold, and if the resulting c[0] exceeds what we already have.
        if ( contact.surface.bounce_vel >= 0 &&
                ( -outgoing ) > contact.surface.bounce_vel )
        {
            dReal newc = - contact.surface.bounce * outgoing + motionN;
            if ( newc > info->c[0] ) info->c[0] = newc;
        }
    }

    // set LCP limits for normal
    info->lo[0] = 0;
    info->hi[0] = dInfinity;
    info->findex[0] = -2;

    // now do jacobian for tangential forces
    dVector3 t1, t2; // two vectors tangential to normal

    // first friction direction
    if ( the_m >= 2 )
    {
        if ( contact.surface.mode & dContactFDir1 )   // use fdir1 ?
        {
            t1[0] = contact.fdir1[0];
            t1[1] = contact.fdir1[1];
            t1[2] = contact.fdir1[2];
            dCalcVectorCross3( t2, normal, t1 );

            // if fdir1 is parallel to normal, revert to dPlaneSpace
            if (_dequal(t2[0], 0.0) &&
                _dequal(t2[1], 0.0) &&
                _dequal(t2[2], 0.0))
              dPlaneSpace( normal, t1, t2 );
        }
        else
        {
            dPlaneSpace( normal, t1, t2 );
        }
        info->J1l[s+0] = t1[0];
        info->J1l[s+1] = t1[1];
        info->J1l[s+2] = t1[2];
        dCalcVectorCross3( info->J1a + s, c1, t1 );
        if ( node[1].body )
        {
            info->J2l[s+0] = -t1[0];
            info->J2l[s+1] = -t1[1];
            info->J2l[s+2] = -t1[2];
            dReal *J2a_plus_s = info->J2a + s;
            dCalcVectorCross3( J2a_plus_s, c2, t1 );
            dNegateVector3( J2a_plus_s );
        }
        // set right hand side
        if ( contact.surface.mode & dContactMotion1 )
        {
            info->c[1] = contact.surface.motion1;
        }
        // set LCP bounds and friction index. this depends on the approximation
        // mode
        info->lo[1] = -contact.surface.mu;
        info->hi[1] = contact.surface.mu;
        if ( contact.surface.mode & dContactApprox1_1 )
            info->findex[1] = 0;

        // set slip (constraint force mixing)
        if ( contact.surface.mode & dContactSlip1 )
            info->cfm[1] = contact.surface.slip1;
    }

    // second friction direction
    if ( the_m >= 3 )
    {
        info->J1l[s2+0] = t2[0];
        info->J1l[s2+1] = t2[1];
        info->J1l[s2+2] = t2[2];
        dCalcVectorCross3( info->J1a + s2, c1, t2 );
        if ( node[1].body )
        {
            info->J2l[s2+0] = -t2[0];
            info->J2l[s2+1] = -t2[1];
            info->J2l[s2+2] = -t2[2];
            dReal *J2a_plus_s2 = info->J2a + s2;
            dCalcVectorCross3( J2a_plus_s2, c2, t2 );
            dNegateVector3( J2a_plus_s2 );
        }
        // set right hand side
        if ( contact.surface.mode & dContactMotion2 )
        {
            info->c[2] = contact.surface.motion2;
        }
        // set LCP bounds and friction index. this depends on the approximation
        // mode
        if ( contact.surface.mode & dContactMu2 )
        {
            info->lo[2] = -contact.surface.mu2;
            info->hi[2] = contact.surface.mu2;
        }
        else
        {
            info->lo[2] = -contact.surface.mu;
            info->hi[2] = contact.surface.mu;
        }
        if ( contact.surface.mode & dContactApprox1_2 )
            info->findex[2] = 0;

        // set slip (constraint force mixing)
        if ( contact.surface.mode & dContactSlip2 )
            info->cfm[2] = contact.surface.slip2;
    }

    // now do jacobian for rotational forces

    // third friction direction (torsional)
    // note that this will only be reachable if mu and mu2
    // have positive values
    if ( the_m >= 4 )
    {
        dVector3 t3 = {0, 0, 0};

        // Linear, body 1
        info->J1l[s3+0] = t3[0];
        info->J1l[s3+1] = t3[1];
        info->J1l[s3+2] = t3[2];

        // Angular, body 1
        info->J1a[s3+0] = normal[0];
        info->J1a[s3+1] = normal[1];
        info->J1a[s3+2] = normal[2];
        if ( node[1].body )
        {
            // Linear, body 2
            info->J2l[s3+0] = -t3[0];
            info->J2l[s3+1] = -t3[1];
            info->J2l[s3+2] = -t3[2];

            // Angular, body 2
            info->J2a[s3+0] = -normal[0];
            info->J2a[s3+1] = -normal[1];
            info->J2a[s3+2] = -normal[2];
        }
        // set LCP bounds and friction index. this depends on the approximation
        // mode
        if ( contact.surface.mode & dContactMu3 )
        {
            // Use user defined torsional patch radius
            //
            // M = torsional moment
            // F = normal force
            // a = patch radius
            // R = surface radius
            // d = depth
            // mu = torsional friction coefficient
            //
            // M = (3 * pi * a * mu3)/16 * F
            //
            // When using radius:
            //
            // a = sqrt (R * d)
            //
            // M = (3 * pi * mu3 * sqrt (R * d))/16 * F

            dReal patch_radius;
            if (!contact.surface.use_patch_radius)
            {
              patch_radius = sqrt(contact.surface.surface_radius * depth);
            }
            else
            {
              patch_radius = contact.surface.patch_radius;
            }

            double rhs = (3 * M_PI * patch_radius * contact.surface.mu3)/16;

            info->lo[3] = -rhs;
            info->hi[3] = rhs;

            // findex[3] must be zero in order for torsional friction moment
            // to be proportional to normal force
            if ( contact.surface.mode & dContactApprox3 )
                info->findex[3] = 0;

            // set slip (constraint force mixing)
            if ( contact.surface.mode & dContactSlip3 )
                info->cfm[3] = contact.surface.slip3;
        }
    }
}

dJointType
dxJointContact::type() const
{
    return dJointTypeContact;
}


size_t
dxJointContact::size() const
{
    return sizeof( *this );
}

