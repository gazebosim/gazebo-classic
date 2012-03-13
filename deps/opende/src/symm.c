/* ilaenv.f -- translated by f2c (version 20090411).
   You must link the resulting object file with libf2c:
   on Microsoft Windows system, link with libf2c.lib;
   on Linux or Unix systems, link with .../path/to/libf2c.a -lm
   or, if you install libf2c.a in a standard place, with -lf2c -lm
   -- in that order, at the end of the command line, as in
   cc *.o -lf2c -lm
   Source for libf2c is in /netlib/f2c/libf2c.zip, e.g.,

http://www.netlib.org/f2c/libf2c.zip
*/

#include <ode/common.h>
#include <math.h>

/* f2c stuff*/
typedef int integer;
typedef dReal real;
typedef short ftnlen;
typedef int logical;
typedef double doublereal;

#define dabs(x) (doublereal)abs(x)
#define min(a,b) ((a) <= (b) ? (a) : (b))
#define max(a,b) ((a) >= (b) ? (a) : (b))
#define dmax(a,b) (doublereal)max(a,b)
#define _dequal(a,b) (fabs(a - b) <= 1e-6)

/* Table of constant values */

static integer c__1 = 1;
static real c_b163 = 0.f;
static real c_b164 = 1.f;
static integer c__0 = 0;

#ifdef __cplusplus
extern "C" {
#endif


  /* assign strings:  a = b */

#ifdef KR_headers
  VOID s_copy(a, b, la, lb) register char *a, *b; ftnlen la, lb;
#else
  void s_copy(register char *a, register char *b, ftnlen la, ftnlen lb)
#endif
  {
    register char *aend, *bend;

    aend = a + la;

    if(la <= lb)
#ifndef NO_OVERWRITE
      if (a <= b || a >= b + la)
#endif
        while(a < aend)
          *a++ = *b++;
#ifndef NO_OVERWRITE
      else
        for(b += la; a < aend; )
          *--aend = *--b;
#endif

    else {
      bend = b + lb;
#ifndef NO_OVERWRITE
      if (a <= b || a >= bend)
#endif
        while(b < bend)
          *a++ = *b++;
#ifndef NO_OVERWRITE
      else {
        a += lb;
        while(b < bend)
          *--a = *--bend;
        a += lb;
      }
#endif
      while(a < aend)
        *a++ = ' ';
    }
  }
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
extern "C" {
#endif

  /* compare two strings */

#ifdef KR_headers
  integer s_cmp(a0, b0, la, lb) char *a0, *b0; ftnlen la, lb;
#else
  integer s_cmp(char *a0, char *b0, ftnlen la, ftnlen lb)
#endif
  {
    register unsigned char *a, *aend, *b, *bend;
    a = (unsigned char *)a0;
    b = (unsigned char *)b0;
    aend = a + la;
    bend = b + lb;

    if(la <= lb)
    {
      while(a < aend)
        if(*a != *b)
          return( *a - *b );
        else
        { ++a; ++b; }

      while(b < bend)
        if(*b != ' ')
          return( ' ' - *b );
        else	++b;
    }

    else
    {
      while(b < bend)
        if(*a == *b)
        { ++a; ++b; }
        else
          return( *a - *b );
      while(a < aend)
        if(*a != ' ')
          return(*a - ' ');
        else	++a;
    }
    return(0);
  }
#ifdef __cplusplus
}
#endif


integer ieeeck_(integer *ispec, real *zero, real *one)
{
  /* System generated locals */
  integer ret_val;

  /* Local variables */
  static real nan1, nan2, nan3, nan4, nan5, nan6, neginf, posinf, negzro, 
              newzro;


  /*  -- LAPACK auxiliary routine (version 3.2) -- */
  /*  -- LAPACK is a software package provided by Univ. of Tennessee,    -- */
  /*  -- Univ. of California Berkeley, Univ. of Colorado Denver and NAG Ltd..-- */
  /*     November 2006 */

  /*     .. Scalar Arguments .. */
  /*     .. */

  /*  Purpose */
  /*  ======= */

  /*  IEEECK is called from the ILAENV to verify that Infinity and */
  /*  possibly NaN arithmetic is safe (i.e. will not trap). */

  /*  Arguments */
  /*  ========= */

  /*  ISPEC   (input) INTEGER */
  /*          Specifies whether to test just for inifinity arithmetic */
  /*          or whether to test for infinity and NaN arithmetic. */
  /*          = 0: Verify infinity arithmetic only. */
  /*          = 1: Verify infinity and NaN arithmetic. */

  /*  ZERO    (input) REAL */
  /*          Must contain the value 0.0 */
  /*          This is passed to prevent the compiler from optimizing */
  /*          away this code. */

  /*  ONE     (input) REAL */
  /*          Must contain the value 1.0 */
  /*          This is passed to prevent the compiler from optimizing */
  /*          away this code. */

  /*  RETURN VALUE:  INTEGER */
  /*          = 0:  Arithmetic failed to produce the correct answers */
  /*          = 1:  Arithmetic produced the correct answers */

  /*     .. Local Scalars .. */
  /*     .. */
  /*     .. Executable Statements .. */
  ret_val = 1;

  posinf = *one / *zero;
  if (posinf <= *one) {
    ret_val = 0;
    return ret_val;
  }

  neginf = -(*one) / *zero;
  if (neginf >= *zero) {
    ret_val = 0;
    return ret_val;
  }

  negzro = *one / (neginf + *one);
  if (!_dequal(negzro, *zero)) {
    ret_val = 0;
    return ret_val;
  }

  neginf = *one / negzro;
  if (neginf >= *zero) {
    ret_val = 0;
    return ret_val;
  }

  newzro = negzro + *zero;
  if (!_dequal(newzro, *zero)) {
    ret_val = 0;
    return ret_val;
  }

  posinf = *one / newzro;
  if (posinf <= *one) {
    ret_val = 0;
    return ret_val;
  }

  neginf *= posinf;
  if (neginf >= *zero) {
    ret_val = 0;
    return ret_val;
  }

  posinf *= posinf;
  if (posinf <= *one) {
    ret_val = 0;
    return ret_val;
  }




  /*     Return if we were only asked to check infinity arithmetic */

  if (*ispec == 0) {
    return ret_val;
  }

  nan1 = posinf + neginf;

  nan2 = posinf / neginf;

  nan3 = posinf / posinf;

  nan4 = posinf * *zero;

  nan5 = neginf * negzro;

  nan6 = nan5 * 0.f;

  if (_dequal(nan1, nan1)) {
    ret_val = 0;
    return ret_val;
  }

  if (_dequal(nan2, nan2)) {
    ret_val = 0;
    return ret_val;
  }

  if (_dequal(nan3, nan3)) {
    ret_val = 0;
    return ret_val;
  }

  if (_dequal(nan4, nan4)) {
    ret_val = 0;
    return ret_val;
  }

  if (_dequal(nan5, nan5)) {
    ret_val = 0;
    return ret_val;
  }

  if (_dequal(nan6, nan6)) {
    ret_val = 0;
    return ret_val;
  }

  return ret_val;
} /* ieeeck_ */

integer ilaenv_(integer *ispec, char *name__, char *opts, integer *n1, 
    integer *n2, integer *n3, integer *n4, ftnlen name_len, ftnlen 
    opts_len)
{
  /* System generated locals */
  integer ret_val;

  /* Local variables */
  static integer i__;
  static char c1[1], c2[2], c3[3], c4[2];
  static integer ic, nb, iz, nx;
  static logical cname;
  static integer nbmin;
  static logical sname;
  extern integer ieeeck_(integer *, real *, real *);
  static char subnam[6];
  extern integer iparmq_(integer *, char *, char *, integer *, integer *, 
      integer *, integer *, ftnlen, ftnlen);


  /*  -- LAPACK auxiliary routine (version 3.2.1)                        -- */

  /*  -- April 2009                                                      -- */

  /*  -- LAPACK is a software package provided by Univ. of Tennessee,    -- */
  /*  -- Univ. of California Berkeley, Univ. of Colorado Denver and NAG Ltd..-- */

  /*     .. Scalar Arguments .. */
  /*     .. */

  /*  Purpose */
  /*  ======= */

  /*  ILAENV is called from the LAPACK routines to choose problem-dependent */
  /*  parameters for the local environment.  See ISPEC for a description of */
  /*  the parameters. */

  /*  ILAENV returns an INTEGER */
  /*  if ILAENV >= 0: ILAENV returns the value of the parameter specified by ISPEC */
  /*  if ILAENV < 0:  if ILAENV = -k, the k-th argument had an illegal value. */

  /*  This version provides a set of parameters which should give good, */
  /*  but not optimal, performance on many of the currently available */
  /*  computers.  Users are encouraged to modify this subroutine to set */
  /*  the tuning parameters for their particular machine using the option */
  /*  and problem size information in the arguments. */

  /*  This routine will not function correctly if it is converted to all */
  /*  lower case.  Converting it to all upper case is allowed. */

  /*  Arguments */
  /*  ========= */

  /*  ISPEC   (input) INTEGER */
  /*          Specifies the parameter to be returned as the value of */
  /*          ILAENV. */
  /*          = 1: the optimal blocksize; if this value is 1, an unblocked */
  /*               algorithm will give the best performance. */
  /*          = 2: the minimum block size for which the block routine */
  /*               should be used; if the usable block size is less than */
  /*               this value, an unblocked routine should be used. */
  /*          = 3: the crossover point (in a block routine, for N less */
  /*               than this value, an unblocked routine should be used) */
  /*          = 4: the number of shifts, used in the nonsymmetric */
  /*               eigenvalue routines (DEPRECATED) */
  /*          = 5: the minimum column dimension for blocking to be used; */
  /*               rectangular blocks must have dimension at least k by m, */
  /*               where k is given by ILAENV(2,...) and m by ILAENV(5,...) */
  /*          = 6: the crossover point for the SVD (when reducing an m by n */
  /*               matrix to bidiagonal form, if max(m,n)/min(m,n) exceeds */
  /*               this value, a QR factorization is used first to reduce */
  /*               the matrix to a triangular form.) */
  /*          = 7: the number of processors */
  /*          = 8: the crossover point for the multishift QR method */
  /*               for nonsymmetric eigenvalue problems (DEPRECATED) */
  /*          = 9: maximum size of the subproblems at the bottom of the */
  /*               computation tree in the divide-and-conquer algorithm */
  /*               (used by xGELSD and xGESDD) */
  /*          =10: ieee NaN arithmetic can be trusted not to trap */
  /*          =11: infinity arithmetic can be trusted not to trap */
  /*          12 <= ISPEC <= 16: */
  /*               xHSEQR or one of its subroutines, */
  /*               see IPARMQ for detailed explanation */

  /*  NAME    (input) CHARACTER*(*) */
  /*          The name of the calling subroutine, in either upper case or */
  /*          lower case. */

  /*  OPTS    (input) CHARACTER*(*) */
  /*          The character options to the subroutine NAME, concatenated */
  /*          into a single character string.  For example, UPLO = 'U', */
  /*          TRANS = 'T', and DIAG = 'N' for a triangular routine would */
  /*          be specified as OPTS = 'UTN'. */

  /*  N1      (input) INTEGER */
  /*  N2      (input) INTEGER */
  /*  N3      (input) INTEGER */
  /*  N4      (input) INTEGER */
  /*          Problem dimensions for the subroutine NAME; these may not all */
  /*          be required. */

  /*  Further Details */
  /*  =============== */

  /*  The following conventions have been used when calling ILAENV from the */
  /*  LAPACK routines: */
  /*  1)  OPTS is a concatenation of all of the character options to */
  /*      subroutine NAME, in the same order that they appear in the */
  /*      argument list for NAME, even if they are not used in determining */
  /*      the value of the parameter specified by ISPEC. */
  /*  2)  The problem dimensions N1, N2, N3, N4 are specified in the order */
  /*      that they appear in the argument list for NAME.  N1 is used */
  /*      first, N2 second, and so on, and unused problem dimensions are */
  /*      passed a value of -1. */
  /*  3)  The parameter value returned by ILAENV is checked for validity in */
  /*      the calling subroutine.  For example, ILAENV is used to retrieve */
  /*      the optimal blocksize for STRTRI as follows: */

  /*      NB = ILAENV( 1, 'STRTRI', UPLO // DIAG, N, -1, -1, -1 ) */
  /*      IF( NB.LE.1 ) NB = MAX( 1, N ) */

  /*  ===================================================================== */

  /*     .. Local Scalars .. */
  /*     .. */
  /*     .. Intrinsic Functions .. */
  /*     .. */
  /*     .. External Functions .. */
  /*     .. */
  /*     .. Executable Statements .. */

  switch (*ispec) {
    case 1:  goto L10;
    case 2:  goto L10;
    case 3:  goto L10;
    case 4:  goto L80;
    case 5:  goto L90;
    case 6:  goto L100;
    case 7:  goto L110;
    case 8:  goto L120;
    case 9:  goto L130;
    case 10:  goto L140;
    case 11:  goto L150;
    case 12:  goto L160;
    case 13:  goto L160;
    case 14:  goto L160;
    case 15:  goto L160;
    case 16:  goto L160;
    default:
              break;
  }

  /*     Invalid value for ISPEC */

  ret_val = -1;
  return ret_val;

L10:

  /*     Convert NAME to upper case if the first character is lower case. */

  ret_val = 1;
  s_copy(subnam, name__, (ftnlen)6, name_len);
  ic = *(unsigned char *)subnam;
  iz = 'Z';
  if (iz == 90 || iz == 122) {

    /*        ASCII character set */

    if (ic >= 97 && ic <= 122) {
      *(unsigned char *)subnam = (char) (ic - 32);
      for (i__ = 2; i__ <= 6; ++i__) {
        ic = *(unsigned char *)&subnam[i__ - 1];
        if (ic >= 97 && ic <= 122) {
          *(unsigned char *)&subnam[i__ - 1] = (char) (ic - 32);
        }
        /* L20: */
      }
    }

  } else if (iz == 233 || iz == 169) {

    /*        EBCDIC character set */

    if ((ic >= 129 && ic <= 137) || (ic >= 145 && ic <= 153) || 
        (ic >= 162 && ic <= 169)) {
      *(unsigned char *)subnam = (char) (ic + 64);
      for (i__ = 2; i__ <= 6; ++i__) {
        ic = *(unsigned char *)&subnam[i__ - 1];
        if ((ic >= 129 && ic <= 137) || (ic >= 145 && ic <= 153) || 
            (ic >= 162 && ic <= 169)) {
          *(unsigned char *)&subnam[i__ - 1] = (char) (ic + 64);
        }
        /* L30: */
      }
    }

  } else if (iz == 218 || iz == 250) {

    /*        Prime machines:  ASCII+128 */

    if (ic >= 225 && ic <= 250) {
      *(unsigned char *)subnam = (char) (ic - 32);
      for (i__ = 2; i__ <= 6; ++i__) {
        ic = *(unsigned char *)&subnam[i__ - 1];
        if (ic >= 225 && ic <= 250) {
          *(unsigned char *)&subnam[i__ - 1] = (char) (ic - 32);
        }
        /* L40: */
      }
    }
  }

  *(unsigned char *)c1 = *(unsigned char *)subnam;
  sname = *(unsigned char *)c1 == 'S' || *(unsigned char *)c1 == 'D';
  cname = *(unsigned char *)c1 == 'C' || *(unsigned char *)c1 == 'Z';
  if (! (cname || sname)) {
    return ret_val;
  }
  s_copy(c2, subnam + 1, (ftnlen)2, (ftnlen)2);
  s_copy(c3, subnam + 3, (ftnlen)3, (ftnlen)3);
  s_copy(c4, c3 + 1, (ftnlen)2, (ftnlen)2);

  switch (*ispec) {
    case 1:  goto L50;
    case 2:  goto L60;
    case 3:  goto L70;
    default: break;
  }

L50:

  /*     ISPEC = 1:  block size */

  /*     In these examples, separate code is provided for setting NB for */
  /*     real and complex.  We assume that NB will take the same value in */
  /*     single or double precision. */

  nb = 1;

  if (s_cmp(c2, "GE", (ftnlen)2, (ftnlen)2) == 0) {
    if (s_cmp(c3, "TRF", (ftnlen)3, (ftnlen)3) == 0) {
      if (sname) {
        nb = 64;
      } else {
        nb = 64;
      }
    } else if (s_cmp(c3, "QRF", (ftnlen)3, (ftnlen)3) == 0 || s_cmp(c3, 
          "RQF", (ftnlen)3, (ftnlen)3) == 0 || s_cmp(c3, "LQF", (ftnlen)
          3, (ftnlen)3) == 0 || s_cmp(c3, "QLF", (ftnlen)3, (ftnlen)3) 
        == 0) {
      if (sname) {
        nb = 32;
      } else {
        nb = 32;
      }
    } else if (s_cmp(c3, "HRD", (ftnlen)3, (ftnlen)3) == 0) {
      if (sname) {
        nb = 32;
      } else {
        nb = 32;
      }
    } else if (s_cmp(c3, "BRD", (ftnlen)3, (ftnlen)3) == 0) {
      if (sname) {
        nb = 32;
      } else {
        nb = 32;
      }
    } else if (s_cmp(c3, "TRI", (ftnlen)3, (ftnlen)3) == 0) {
      if (sname) {
        nb = 64;
      } else {
        nb = 64;
      }
    }
  } else if (s_cmp(c2, "PO", (ftnlen)2, (ftnlen)2) == 0) {
    if (s_cmp(c3, "TRF", (ftnlen)3, (ftnlen)3) == 0) {
      if (sname) {
        nb = 64;
      } else {
        nb = 64;
      }
    }
  } else if (s_cmp(c2, "SY", (ftnlen)2, (ftnlen)2) == 0) {
    if (s_cmp(c3, "TRF", (ftnlen)3, (ftnlen)3) == 0) {
      if (sname) {
        nb = 64;
      } else {
        nb = 64;
      }
    } else if (sname && s_cmp(c3, "TRD", (ftnlen)3, (ftnlen)3) == 0) {
      nb = 32;
    } else if (sname && s_cmp(c3, "GST", (ftnlen)3, (ftnlen)3) == 0) {
      nb = 64;
    }
  } else if (cname && s_cmp(c2, "HE", (ftnlen)2, (ftnlen)2) == 0) {
    if (s_cmp(c3, "TRF", (ftnlen)3, (ftnlen)3) == 0) {
      nb = 64;
    } else if (s_cmp(c3, "TRD", (ftnlen)3, (ftnlen)3) == 0) {
      nb = 32;
    } else if (s_cmp(c3, "GST", (ftnlen)3, (ftnlen)3) == 0) {
      nb = 64;
    }
  } else if (sname && s_cmp(c2, "OR", (ftnlen)2, (ftnlen)2) == 0) {
    if (*(unsigned char *)c3 == 'G') {
      if (s_cmp(c4, "QR", (ftnlen)2, (ftnlen)2) == 0 || s_cmp(c4, "RQ", 
            (ftnlen)2, (ftnlen)2) == 0 || s_cmp(c4, "LQ", (ftnlen)2, (
              ftnlen)2) == 0 || s_cmp(c4, "QL", (ftnlen)2, (ftnlen)2) ==
          0 || s_cmp(c4, "HR", (ftnlen)2, (ftnlen)2) == 0 || s_cmp(
            c4, "TR", (ftnlen)2, (ftnlen)2) == 0 || s_cmp(c4, "BR", (
              ftnlen)2, (ftnlen)2) == 0) {
        nb = 32;
      }
    } else if (*(unsigned char *)c3 == 'M') {
      if (s_cmp(c4, "QR", (ftnlen)2, (ftnlen)2) == 0 || s_cmp(c4, "RQ", 
            (ftnlen)2, (ftnlen)2) == 0 || s_cmp(c4, "LQ", (ftnlen)2, (
              ftnlen)2) == 0 || s_cmp(c4, "QL", (ftnlen)2, (ftnlen)2) ==
          0 || s_cmp(c4, "HR", (ftnlen)2, (ftnlen)2) == 0 || s_cmp(
            c4, "TR", (ftnlen)2, (ftnlen)2) == 0 || s_cmp(c4, "BR", (
              ftnlen)2, (ftnlen)2) == 0) {
        nb = 32;
      }
    }
  } else if (cname && s_cmp(c2, "UN", (ftnlen)2, (ftnlen)2) == 0) {
    if (*(unsigned char *)c3 == 'G') {
      if (s_cmp(c4, "QR", (ftnlen)2, (ftnlen)2) == 0 || s_cmp(c4, "RQ", 
            (ftnlen)2, (ftnlen)2) == 0 || s_cmp(c4, "LQ", (ftnlen)2, (
              ftnlen)2) == 0 || s_cmp(c4, "QL", (ftnlen)2, (ftnlen)2) ==
          0 || s_cmp(c4, "HR", (ftnlen)2, (ftnlen)2) == 0 || s_cmp(
            c4, "TR", (ftnlen)2, (ftnlen)2) == 0 || s_cmp(c4, "BR", (
              ftnlen)2, (ftnlen)2) == 0) {
        nb = 32;
      }
    } else if (*(unsigned char *)c3 == 'M') {
      if (s_cmp(c4, "QR", (ftnlen)2, (ftnlen)2) == 0 || s_cmp(c4, "RQ", 
            (ftnlen)2, (ftnlen)2) == 0 || s_cmp(c4, "LQ", (ftnlen)2, (
              ftnlen)2) == 0 || s_cmp(c4, "QL", (ftnlen)2, (ftnlen)2) ==
          0 || s_cmp(c4, "HR", (ftnlen)2, (ftnlen)2) == 0 || s_cmp(
            c4, "TR", (ftnlen)2, (ftnlen)2) == 0 || s_cmp(c4, "BR", (
              ftnlen)2, (ftnlen)2) == 0) {
        nb = 32;
      }
    }
  } else if (s_cmp(c2, "GB", (ftnlen)2, (ftnlen)2) == 0) {
    if (s_cmp(c3, "TRF", (ftnlen)3, (ftnlen)3) == 0) {
      if (sname) {
        if (*n4 <= 64) {
          nb = 1;
        } else {
          nb = 32;
        }
      } else {
        if (*n4 <= 64) {
          nb = 1;
        } else {
          nb = 32;
        }
      }
    }
  } else if (s_cmp(c2, "PB", (ftnlen)2, (ftnlen)2) == 0) {
    if (s_cmp(c3, "TRF", (ftnlen)3, (ftnlen)3) == 0) {
      if (sname) {
        if (*n2 <= 64) {
          nb = 1;
        } else {
          nb = 32;
        }
      } else {
        if (*n2 <= 64) {
          nb = 1;
        } else {
          nb = 32;
        }
      }
    }
  } else if (s_cmp(c2, "TR", (ftnlen)2, (ftnlen)2) == 0) {
    if (s_cmp(c3, "TRI", (ftnlen)3, (ftnlen)3) == 0) {
      if (sname) {
        nb = 64;
      } else {
        nb = 64;
      }
    }
  } else if (s_cmp(c2, "LA", (ftnlen)2, (ftnlen)2) == 0) {
    if (s_cmp(c3, "UUM", (ftnlen)3, (ftnlen)3) == 0) {
      if (sname) {
        nb = 64;
      } else {
        nb = 64;
      }
    }
  } else if (sname && s_cmp(c2, "ST", (ftnlen)2, (ftnlen)2) == 0) {
    if (s_cmp(c3, "EBZ", (ftnlen)3, (ftnlen)3) == 0) {
      nb = 1;
    }
  }
  ret_val = nb;
  return ret_val;

L60:

  /*     ISPEC = 2:  minimum block size */

  nbmin = 2;
  if (s_cmp(c2, "GE", (ftnlen)2, (ftnlen)2) == 0) {
    if (s_cmp(c3, "QRF", (ftnlen)3, (ftnlen)3) == 0 || s_cmp(c3, "RQF", (
            ftnlen)3, (ftnlen)3) == 0 || s_cmp(c3, "LQF", (ftnlen)3, (
              ftnlen)3) == 0 || s_cmp(c3, "QLF", (ftnlen)3, (ftnlen)3) == 0)
    {
      if (sname) {
        nbmin = 2;
      } else {
        nbmin = 2;
      }
    } else if (s_cmp(c3, "HRD", (ftnlen)3, (ftnlen)3) == 0) {
      if (sname) {
        nbmin = 2;
      } else {
        nbmin = 2;
      }
    } else if (s_cmp(c3, "BRD", (ftnlen)3, (ftnlen)3) == 0) {
      if (sname) {
        nbmin = 2;
      } else {
        nbmin = 2;
      }
    } else if (s_cmp(c3, "TRI", (ftnlen)3, (ftnlen)3) == 0) {
      if (sname) {
        nbmin = 2;
      } else {
        nbmin = 2;
      }
    }
  } else if (s_cmp(c2, "SY", (ftnlen)2, (ftnlen)2) == 0) {
    if (s_cmp(c3, "TRF", (ftnlen)3, (ftnlen)3) == 0) {
      if (sname) {
        nbmin = 8;
      } else {
        nbmin = 8;
      }
    } else if (sname && s_cmp(c3, "TRD", (ftnlen)3, (ftnlen)3) == 0) {
      nbmin = 2;
    }
  } else if (cname && s_cmp(c2, "HE", (ftnlen)2, (ftnlen)2) == 0) {
    if (s_cmp(c3, "TRD", (ftnlen)3, (ftnlen)3) == 0) {
      nbmin = 2;
    }
  } else if (sname && s_cmp(c2, "OR", (ftnlen)2, (ftnlen)2) == 0) {
    if (*(unsigned char *)c3 == 'G') {
      if (s_cmp(c4, "QR", (ftnlen)2, (ftnlen)2) == 0 || s_cmp(c4, "RQ", 
            (ftnlen)2, (ftnlen)2) == 0 || s_cmp(c4, "LQ", (ftnlen)2, (
              ftnlen)2) == 0 || s_cmp(c4, "QL", (ftnlen)2, (ftnlen)2) ==
          0 || s_cmp(c4, "HR", (ftnlen)2, (ftnlen)2) == 0 || s_cmp(
            c4, "TR", (ftnlen)2, (ftnlen)2) == 0 || s_cmp(c4, "BR", (
              ftnlen)2, (ftnlen)2) == 0) {
        nbmin = 2;
      }
    } else if (*(unsigned char *)c3 == 'M') {
      if (s_cmp(c4, "QR", (ftnlen)2, (ftnlen)2) == 0 || s_cmp(c4, "RQ", 
            (ftnlen)2, (ftnlen)2) == 0 || s_cmp(c4, "LQ", (ftnlen)2, (
              ftnlen)2) == 0 || s_cmp(c4, "QL", (ftnlen)2, (ftnlen)2) ==
          0 || s_cmp(c4, "HR", (ftnlen)2, (ftnlen)2) == 0 || s_cmp(
            c4, "TR", (ftnlen)2, (ftnlen)2) == 0 || s_cmp(c4, "BR", (
              ftnlen)2, (ftnlen)2) == 0) {
        nbmin = 2;
      }
    }
  } else if (cname && s_cmp(c2, "UN", (ftnlen)2, (ftnlen)2) == 0) {
    if (*(unsigned char *)c3 == 'G') {
      if (s_cmp(c4, "QR", (ftnlen)2, (ftnlen)2) == 0 || s_cmp(c4, "RQ", 
            (ftnlen)2, (ftnlen)2) == 0 || s_cmp(c4, "LQ", (ftnlen)2, (
              ftnlen)2) == 0 || s_cmp(c4, "QL", (ftnlen)2, (ftnlen)2) ==
          0 || s_cmp(c4, "HR", (ftnlen)2, (ftnlen)2) == 0 || s_cmp(
            c4, "TR", (ftnlen)2, (ftnlen)2) == 0 || s_cmp(c4, "BR", (
              ftnlen)2, (ftnlen)2) == 0) {
        nbmin = 2;
      }
    } else if (*(unsigned char *)c3 == 'M') {
      if (s_cmp(c4, "QR", (ftnlen)2, (ftnlen)2) == 0 || s_cmp(c4, "RQ", 
            (ftnlen)2, (ftnlen)2) == 0 || s_cmp(c4, "LQ", (ftnlen)2, (
              ftnlen)2) == 0 || s_cmp(c4, "QL", (ftnlen)2, (ftnlen)2) ==
          0 || s_cmp(c4, "HR", (ftnlen)2, (ftnlen)2) == 0 || s_cmp(
            c4, "TR", (ftnlen)2, (ftnlen)2) == 0 || s_cmp(c4, "BR", (
              ftnlen)2, (ftnlen)2) == 0) {
        nbmin = 2;
      }
    }
  }
  ret_val = nbmin;
  return ret_val;

L70:

  /*     ISPEC = 3:  crossover point */

  nx = 0;
  if (s_cmp(c2, "GE", (ftnlen)2, (ftnlen)2) == 0) {
    if (s_cmp(c3, "QRF", (ftnlen)3, (ftnlen)3) == 0 || s_cmp(c3, "RQF", (
            ftnlen)3, (ftnlen)3) == 0 || s_cmp(c3, "LQF", (ftnlen)3, (
              ftnlen)3) == 0 || s_cmp(c3, "QLF", (ftnlen)3, (ftnlen)3) == 0)
    {
      if (sname) {
        nx = 128;
      } else {
        nx = 128;
      }
    } else if (s_cmp(c3, "HRD", (ftnlen)3, (ftnlen)3) == 0) {
      if (sname) {
        nx = 128;
      } else {
        nx = 128;
      }
    } else if (s_cmp(c3, "BRD", (ftnlen)3, (ftnlen)3) == 0) {
      if (sname) {
        nx = 128;
      } else {
        nx = 128;
      }
    }
  } else if (s_cmp(c2, "SY", (ftnlen)2, (ftnlen)2) == 0) {
    if (sname && s_cmp(c3, "TRD", (ftnlen)3, (ftnlen)3) == 0) {
      nx = 32;
    }
  } else if (cname && s_cmp(c2, "HE", (ftnlen)2, (ftnlen)2) == 0) {
    if (s_cmp(c3, "TRD", (ftnlen)3, (ftnlen)3) == 0) {
      nx = 32;
    }
  } else if (sname && s_cmp(c2, "OR", (ftnlen)2, (ftnlen)2) == 0) {
    if (*(unsigned char *)c3 == 'G') {
      if (s_cmp(c4, "QR", (ftnlen)2, (ftnlen)2) == 0 || s_cmp(c4, "RQ", 
            (ftnlen)2, (ftnlen)2) == 0 || s_cmp(c4, "LQ", (ftnlen)2, (
              ftnlen)2) == 0 || s_cmp(c4, "QL", (ftnlen)2, (ftnlen)2) ==
          0 || s_cmp(c4, "HR", (ftnlen)2, (ftnlen)2) == 0 || s_cmp(
            c4, "TR", (ftnlen)2, (ftnlen)2) == 0 || s_cmp(c4, "BR", (
              ftnlen)2, (ftnlen)2) == 0) {
        nx = 128;
      }
    }
  } else if (cname && s_cmp(c2, "UN", (ftnlen)2, (ftnlen)2) == 0) {
    if (*(unsigned char *)c3 == 'G') {
      if (s_cmp(c4, "QR", (ftnlen)2, (ftnlen)2) == 0 || s_cmp(c4, "RQ", 
            (ftnlen)2, (ftnlen)2) == 0 || s_cmp(c4, "LQ", (ftnlen)2, (
              ftnlen)2) == 0 || s_cmp(c4, "QL", (ftnlen)2, (ftnlen)2) ==
          0 || s_cmp(c4, "HR", (ftnlen)2, (ftnlen)2) == 0 || s_cmp(
            c4, "TR", (ftnlen)2, (ftnlen)2) == 0 || s_cmp(c4, "BR", (
              ftnlen)2, (ftnlen)2) == 0) {
        nx = 128;
      }
    }
  }
  ret_val = nx;
  return ret_val;

L80:

  /*     ISPEC = 4:  number of shifts (used by xHSEQR) */

  ret_val = 6;
  return ret_val;

L90:

  /*     ISPEC = 5:  minimum column dimension (not used) */

  ret_val = 2;
  return ret_val;

L100:

  /*     ISPEC = 6:  crossover point for SVD (used by xGELSS and xGESVD) */

  ret_val = (integer) ((real) min(*n1,*n2) * 1.6f);
  return ret_val;

L110:

  /*     ISPEC = 7:  number of processors (not used) */

  ret_val = 1;
  return ret_val;

L120:

  /*     ISPEC = 8:  crossover point for multishift (used by xHSEQR) */

  ret_val = 50;
  return ret_val;

L130:

  /*     ISPEC = 9:  maximum size of the subproblems at the bottom of the */
  /*                 computation tree in the divide-and-conquer algorithm */
  /*                 (used by xGELSD and xGESDD) */

  ret_val = 25;
  return ret_val;

L140:

  /*     ISPEC = 10: ieee NaN arithmetic can be trusted not to trap */

  /*     ILAENV = 0 */
  ret_val = 1;
  if (ret_val == 1) {
    ret_val = ieeeck_(&c__1, &c_b163, &c_b164);
  }
  return ret_val;

L150:

  /*     ISPEC = 11: infinity arithmetic can be trusted not to trap */

  /*     ILAENV = 0 */
  ret_val = 1;
  if (ret_val == 1) {
    ret_val = ieeeck_(&c__0, &c_b163, &c_b164);
  }
  return ret_val;

L160:

  /*     12 <= ISPEC <= 16: xHSEQR or one of its subroutines. */

  ret_val = iparmq_(ispec, name__, opts, n1, n2, n3, n4, name_len, opts_len)
    ;
  return ret_val;

  /*     End of ILAENV */

} /* ilaenv_ */

#ifdef KR_headers
double floor();
integer i_nint(x) real *x;
#else
#undef abs
#include "math.h"
#ifdef __cplusplus
extern "C" {
#endif
  integer i_nint(real *x)
#endif
  {
    return (integer)(*x >= 0 ? floor(*x + .5) : -floor(.5 - *x));
  }
#ifdef __cplusplus
}
#endif

integer iparmq_(integer *ispec, char * name__, char * opts, integer *n, integer 
    *ilo, integer *ihi, integer * lwork, ftnlen name_len, ftnlen opts_len)
{
  /* System generated locals */
  integer ret_val, i__1, i__2;
  real r__1;

  /* Builtin functions */
  double log(doublereal);
  integer i_nint(real *);

  /* Local variables */
  static integer nh, ns;

  (void)(name__);
  (void)(n);
  (void)(opts);
  (void)(lwork);
  (void)(name_len);
  (void)(opts_len);



  /*  -- LAPACK auxiliary routine (version 3.2) -- */
  /*  -- LAPACK is a software package provided by Univ. of Tennessee,    -- */
  /*  -- Univ. of California Berkeley, Univ. of Colorado Denver and NAG Ltd..-- */
  /*     November 2006 */

  /*     .. Scalar Arguments .. */

  /*  Purpose */
  /*  ======= */

  /*       This program sets problem and machine dependent parameters */
  /*       useful for xHSEQR and its subroutines. It is called whenever */
  /*       ILAENV is called with 12 <= ISPEC <= 16 */

  /*  Arguments */
  /*  ========= */

  /*       ISPEC  (input) integer scalar */
  /*              ISPEC specifies which tunable parameter IPARMQ should */
  /*              return. */

  /*              ISPEC=12: (INMIN)  Matrices of order nmin or less */
  /*                        are sent directly to xLAHQR, the implicit */
  /*                        double shift QR algorithm.  NMIN must be */
  /*                        at least 11. */

  /*              ISPEC=13: (INWIN)  Size of the deflation window. */
  /*                        This is best set greater than or equal to */
  /*                        the number of simultaneous shifts NS. */
  /*                        Larger matrices benefit from larger deflation */
  /*                        windows. */

  /*              ISPEC=14: (INIBL) Determines when to stop nibbling and */
  /*                        invest in an (expensive) multi-shift QR sweep. */
  /*                        If the aggressive early deflation subroutine */
  /*                        finds LD converged eigenvalues from an order */
  /*                        NW deflation window and LD.GT.(NW*NIBBLE)/100, */
  /*                        then the next QR sweep is skipped and early */
  /*                        deflation is applied immediately to the */
  /*                        remaining active diagonal block.  Setting */
  /*                        IPARMQ(ISPEC=14) = 0 causes TTQRE to skip a */
  /*                        multi-shift QR sweep whenever early deflation */
  /*                        finds a converged eigenvalue.  Setting */
  /*                        IPARMQ(ISPEC=14) greater than or equal to 100 */
  /*                        prevents TTQRE from skipping a multi-shift */
  /*                        QR sweep. */

  /*              ISPEC=15: (NSHFTS) The number of simultaneous shifts in */
  /*                        a multi-shift QR iteration. */

  /*              ISPEC=16: (IACC22) IPARMQ is set to 0, 1 or 2 with the */
  /*                        following meanings. */
  /*                        0:  During the multi-shift QR sweep, */
  /*                            xLAQR5 does not accumulate reflections and */
  /*                            does not use matrix-matrix multiply to */
  /*                            update the far-from-diagonal matrix */
  /*                            entries. */
  /*                        1:  During the multi-shift QR sweep, */
  /*                            xLAQR5 and/or xLAQRaccumulates reflections and uses */
  /*                            matrix-matrix multiply to update the */
  /*                            far-from-diagonal matrix entries. */
  /*                        2:  During the multi-shift QR sweep. */
  /*                            xLAQR5 accumulates reflections and takes */
  /*                            advantage of 2-by-2 block structure during */
  /*                            matrix-matrix multiplies. */
  /*                        (If xTRMM is slower than xGEMM, then */
  /*                        IPARMQ(ISPEC=16)=1 may be more efficient than */
  /*                        IPARMQ(ISPEC=16)=2 despite the greater level of */
  /*                        arithmetic work implied by the latter choice.) */

  /*       NAME    (input) character string */
  /*               Name of the calling subroutine */

  /*       OPTS    (input) character string */
  /*               This is a concatenation of the string arguments to */
  /*               TTQRE. */

  /*       N       (input) integer scalar */
  /*               N is the order of the Hessenberg matrix H. */

  /*       ILO     (input) INTEGER */
  /*       IHI     (input) INTEGER */
  /*               It is assumed that H is already upper triangular */
  /*               in rows and columns 1:ILO-1 and IHI+1:N. */

  /*       LWORK   (input) integer scalar */
  /*               The amount of workspace available. */

  /*  Further Details */
  /*  =============== */

  /*       Little is known about how best to choose these parameters. */
  /*       It is possible to use different values of the parameters */
  /*       for each of CHSEQR, DHSEQR, SHSEQR and ZHSEQR. */

  /*       It is probably best to choose different parameters for */
  /*       different matrices and different parameters at different */
  /*       times during the iteration, but this has not been */
  /*       implemented --- yet. */


  /*       The best choices of most of the parameters depend */
  /*       in an ill-understood way on the relative execution */
  /*       rate of xLAQR3 and xLAQR5 and on the nature of each */
  /*       particular eigenvalue problem.  Experiment may be the */
  /*       only practical way to determine which choices are most */
  /*       effective. */

  /*       Following is a list of default values supplied by IPARMQ. */
  /*       These defaults may be adjusted in order to attain better */
  /*       performance in any particular computational environment. */

  /*       IPARMQ(ISPEC=12) The xLAHQR vs xLAQR0 crossover point. */
  /*                        Default: 75. (Must be at least 11.) */

  /*       IPARMQ(ISPEC=13) Recommended deflation window size. */
  /*                        This depends on ILO, IHI and NS, the */
  /*                        number of simultaneous shifts returned */
  /*                        by IPARMQ(ISPEC=15).  The default for */
  /*                        (IHI-ILO+1).LE.500 is NS.  The default */
  /*                        for (IHI-ILO+1).GT.500 is 3*NS/2. */

  /*       IPARMQ(ISPEC=14) Nibble crossover point.  Default: 14. */

  /*       IPARMQ(ISPEC=15) Number of simultaneous shifts, NS. */
  /*                        a multi-shift QR iteration. */

  /*                        If IHI-ILO+1 is ... */

  /*                        greater than      ...but less    ... the */
  /*                        or equal to ...      than        default is */

  /*                                0               30       NS =   2+ */
  /*                               30               60       NS =   4+ */
  /*                               60              150       NS =  10 */
  /*                              150              590       NS =  ** */
  /*                              590             3000       NS =  64 */
  /*                             3000             6000       NS = 128 */
  /*                             6000             infinity   NS = 256 */

  /*                    (+)  By default matrices of this order are */
  /*                         passed to the implicit double shift routine */
  /*                         xLAHQR.  See IPARMQ(ISPEC=12) above.   These */
  /*                         values of NS are used only in case of a rare */
  /*                         xLAHQR failure. */

  /*                    (**) The asterisks (**) indicate an ad-hoc */
  /*                         function increasing from 10 to 64. */

  /*       IPARMQ(ISPEC=16) Select structured matrix multiply. */
  /*                        (See ISPEC=16 above for details.) */
  /*                        Default: 3. */

  /*     ================================================================ */
  /*     .. Parameters .. */
  /*     .. */
  /*     .. Local Scalars .. */
  /*     .. */
  /*     .. Intrinsic Functions .. */
  /*     .. */
  /*     .. Executable Statements .. */
  if (*ispec == 15 || *ispec == 13 || *ispec == 16) {

    /*        ==== Set the number simultaneous shifts ==== */

    nh = *ihi - *ilo + 1;
    ns = 2;
    if (nh >= 30) {
      ns = 4;
    }
    if (nh >= 60) {
      ns = 10;
    }
    if (nh >= 150) {
      /* Computing MAX */
      r__1 = log((real) nh) / log(2.f);
      i__1 = 10, i__2 = nh / i_nint(&r__1);
      ns = max(i__1,i__2);
    }
    if (nh >= 590) {
      ns = 64;
    }
    if (nh >= 3000) {
      ns = 128;
    }
    if (nh >= 6000) {
      ns = 256;
    }
    /* Computing MAX */
    i__1 = 2, i__2 = ns - ns % 2;
    ns = max(i__1,i__2);
  }

  if (*ispec == 12) {


    /*        ===== Matrices of order smaller than NMIN get sent */
    /*        .     to xLAHQR, the classic double shift algorithm. */
    /*        .     This must be at least 11. ==== */

    ret_val = 75;

  } else if (*ispec == 14) {

    /*        ==== INIBL: skip a multi-shift qr iteration and */
    /*        .    whenever aggressive early deflation finds */
    /*        .    at least (NIBBLE*(window size)/100) deflations. ==== */

    ret_val = 14;

  } else if (*ispec == 15) {

    /*        ==== NSHFTS: The number of simultaneous shifts ===== */

    ret_val = ns;

  } else if (*ispec == 13) {

    /*        ==== NW: deflation window size.  ==== */

    if (nh <= 500) {
      ret_val = ns;
    } else {
      ret_val = ns * 3 / 2;
    }

  } else if (*ispec == 16) {

    /*        ==== IACC22: Whether to accumulate reflections */
    /*        .     before updating the far-from-diagonal elements */
    /*        .     and whether to use 2-by-2 block structure while */
    /*        .     doing it.  A small amount of work could be saved */
    /*        .     by making this choice dependent also upon the */
    /*        .     NH=IHI-ILO+1. */

    ret_val = 0;
    if (ns >= 14) {
      ret_val = 1;
    }
    if (ns >= 14) {
      ret_val = 2;
    }

  } else {
    /*        ===== invalid value of ispec ===== */
    ret_val = -1;

  }

  /*     ==== End of IPARMQ ==== */

  return ret_val;
} /* iparmq_ */


/* ../BLAS/SRC/isamax.f -- translated by f2c (version 20090411).
   You must link the resulting object file with libf2c:
   on Microsoft Windows system, link with libf2c.lib;
   on Linux or Unix systems, link with .../path/to/libf2c.a -lm
   or, if you install libf2c.a in a standard place, with -lf2c -lm
   -- in that order, at the end of the command line, as in
   cc *.o -lf2c -lm
   Source for libf2c is in /netlib/f2c/libf2c.zip, e.g.,

http://www.netlib.org/f2c/libf2c.zip
*/

integer isamax_(integer *n, real *sx, integer *incx)
{
  /* System generated locals */
  integer ret_val, i__1;
  real r__1;

  /* Local variables */
  static integer i__, ix;
  static real smax;

  /*     .. Scalar Arguments .. */
  /*     .. */
  /*     .. Array Arguments .. */
  /*     .. */

  /*  Purpose */
  /*  ======= */

  /*     ISAMAX finds the index of element having max. absolute value. */

  /*  Further Details */
  /*  =============== */

  /*     jack dongarra, linpack, 3/11/78. */
  /*     modified 3/93 to return if incx .le. 0. */
  /*     modified 12/3/93, array(1) declarations changed to array(*) */

  /*  ===================================================================== */

  /*     .. Local Scalars .. */
  /*     .. */
  /*     .. Intrinsic Functions .. */
  /*     .. */
  /* Parameter adjustments */
  --sx;

  /* Function Body */
  ret_val = 0;
  if (*n < 1 || *incx <= 0) {
    return ret_val;
  }
  ret_val = 1;
  if (*n == 1) {
    return ret_val;
  }
  if (*incx == 1) {
    goto L20;
  }

  /*        code for increment not equal to 1 */

  ix = 1;
  smax = dabs(sx[1]);
  ix += *incx;
  i__1 = *n;
  for (i__ = 2; i__ <= i__1; ++i__) {
    if ((r__1 = sx[ix], dabs(r__1)) <= smax) {
      goto L5;
    }
    ret_val = i__;
    smax = (r__1 = sx[ix], dabs(r__1));
L5:
    ix += *incx;
    /* L10: */
  }
  return ret_val;

  /*        code for increment equal to 1 */

L20:
  smax = dabs(sx[1]);
  i__1 = *n;
  for (i__ = 2; i__ <= i__1; ++i__) {
    if ((r__1 = sx[i__], dabs(r__1)) <= smax) {
      goto L30;
    }
    ret_val = i__;
    smax = (r__1 = sx[i__], dabs(r__1));
L30:
    ;
  }
  return ret_val;
} /* isamax_ */

/* ../BLAS/SRC/lsame.f -- translated by f2c (version 20090411).
   You must link the resulting object file with libf2c:
   on Microsoft Windows system, link with libf2c.lib;
   on Linux or Unix systems, link with .../path/to/libf2c.a -lm
   or, if you install libf2c.a in a standard place, with -lf2c -lm
   -- in that order, at the end of the command line, as in
   cc *.o -lf2c -lm
   Source for libf2c is in /netlib/f2c/libf2c.zip, e.g.,

http://www.netlib.org/f2c/libf2c.zip
*/

logical lsame_(char *ca, char *cb, ftnlen ca_len, ftnlen cb_len)
{
  /* System generated locals */
  logical ret_val;

  /* Local variables */
  static integer inta, intb, zcode;

  (void)(ca_len);
  (void)(cb_len);



  /*  -- LAPACK auxiliary routine (version 3.1) -- */
  /*     Univ. of Tennessee, Univ. of California Berkeley and NAG Ltd.. */
  /*     November 2006 */

  /*     .. Scalar Arguments .. */
  /*     .. */

  /*  Purpose */
  /*  ======= */

  /*  LSAME returns .TRUE. if CA is the same letter as CB regardless of */
  /*  case. */

  /*  Arguments */
  /*  ========= */

  /*  CA      (input) CHARACTER*1 */

  /*  CB      (input) CHARACTER*1 */
  /*          CA and CB specify the single characters to be compared. */

  /* ===================================================================== */

  /*     .. Intrinsic Functions .. */
  /*     .. */
  /*     .. Local Scalars .. */
  /*     .. */

  /*     Test if the characters are equal */

  ret_val = *(unsigned char *)ca == *(unsigned char *)cb;
  if (ret_val) {
    return ret_val;
  }

  /*     Now test for equivalence if both characters are alphabetic. */

  zcode = 'Z';

  /*     Use 'Z' rather than 'A' so that ASCII can be detected on Prime */
  /*     machines, on which ICHAR returns a value with bit 8 set. */
  /*     ICHAR('A') on Prime machines returns 193 which is the same as */
  /*     ICHAR('A') on an EBCDIC machine. */

  inta = *(unsigned char *)ca;
  intb = *(unsigned char *)cb;

  if (zcode == 90 || zcode == 122) {

    /*        ASCII is assumed - ZCODE is the ASCII code of either lower or */
    /*        upper case 'Z'. */

    if (inta >= 97 && inta <= 122) {
      inta += -32;
    }
    if (intb >= 97 && intb <= 122) {
      intb += -32;
    }

  } else if (zcode == 233 || zcode == 169) {

    /*        EBCDIC is assumed - ZCODE is the EBCDIC code of either lower or */
    /*        upper case 'Z'. */

    if ((inta >= 129 && inta <= 137) || (inta >= 145 && inta <= 153) || 
        (inta >= 162 && inta <= 169)) {
      inta += 64;
    }
    if ((intb >= 129 && intb <= 137) || (intb >= 145 && intb <= 153) || 
        (intb >= 162 && intb <= 169)) {
      intb += 64;
    }

  } else if (zcode == 218 || zcode == 250) {

    /*        ASCII is assumed, on Prime machines - ZCODE is the ASCII code */
    /*        plus 128 of either lower or upper case 'Z'. */

    if (inta >= 225 && inta <= 250) {
      inta += -32;
    }
    if (intb >= 225 && intb <= 250) {
      intb += -32;
    }
  }
  ret_val = inta == intb;

  /*     RETURN */

  /*     End of LSAME */

  return ret_val;
} /* lsame_ */

/* ../BLAS/SRC/scopy.f -- translated by f2c (version 20090411).
   You must link the resulting object file with libf2c:
   on Microsoft Windows system, link with libf2c.lib;
   on Linux or Unix systems, link with .../path/to/libf2c.a -lm
   or, if you install libf2c.a in a standard place, with -lf2c -lm
   -- in that order, at the end of the command line, as in
   cc *.o -lf2c -lm
   Source for libf2c is in /netlib/f2c/libf2c.zip, e.g.,

http://www.netlib.org/f2c/libf2c.zip
*/

/* Subroutine */ int scopy_(integer *n, real *sx, integer *incx, real *sy, 
    integer *incy)
{
  /* System generated locals */
  integer i__1;

  /* Local variables */
  static integer i__, m, ix, iy, mp1;

  /*     .. Scalar Arguments .. */
  /*     .. */
  /*     .. Array Arguments .. */
  /*     .. */

  /*  Purpose */
  /*  ======= */

  /*     SCOPY copies a vector, x, to a vector, y. */
  /*     uses unrolled loops for increments equal to 1. */

  /*  Further Details */
  /*  =============== */

  /*     jack dongarra, linpack, 3/11/78. */
  /*     modified 12/3/93, array(1) declarations changed to array(*) */

  /*  ===================================================================== */

  /*     .. Local Scalars .. */
  /*     .. */
  /*     .. Intrinsic Functions .. */
  /*     .. */
  /* Parameter adjustments */
  --sy;
  --sx;

  /* Function Body */
  if (*n <= 0) {
    return 0;
  }
  if (*incx == 1 && *incy == 1) {
    goto L20;
  }

  /*        code for unequal increments or equal increments */
  /*          not equal to 1 */

  ix = 1;
  iy = 1;
  if (*incx < 0) {
    ix = (-(*n) + 1) * *incx + 1;
  }
  if (*incy < 0) {
    iy = (-(*n) + 1) * *incy + 1;
  }
  i__1 = *n;
  for (i__ = 1; i__ <= i__1; ++i__) {
    sy[iy] = sx[ix];
    ix += *incx;
    iy += *incy;
    /* L10: */
  }
  return 0;

  /*        code for both increments equal to 1 */


  /*        clean-up loop */

L20:
  m = *n % 7;
  if (m == 0) {
    goto L40;
  }
  i__1 = m;
  for (i__ = 1; i__ <= i__1; ++i__) {
    sy[i__] = sx[i__];
    /* L30: */
  }
  if (*n < 7) {
    return 0;
  }
L40:
  mp1 = m + 1;
  i__1 = *n;
  for (i__ = mp1; i__ <= i__1; i__ += 7) {
    sy[i__] = sx[i__];
    sy[i__ + 1] = sx[i__ + 1];
    sy[i__ + 2] = sx[i__ + 2];
    sy[i__ + 3] = sx[i__ + 3];
    sy[i__ + 4] = sx[i__ + 4];
    sy[i__ + 5] = sx[i__ + 5];
    sy[i__ + 6] = sx[i__ + 6];
    /* L50: */
  }
  return 0;
} /* scopy_ */

/* ../BLAS/SRC/sgemm.f -- translated by f2c (version 20090411).
   You must link the resulting object file with libf2c:
   on Microsoft Windows system, link with libf2c.lib;
   on Linux or Unix systems, link with .../path/to/libf2c.a -lm
   or, if you install libf2c.a in a standard place, with -lf2c -lm
   -- in that order, at the end of the command line, as in
   cc *.o -lf2c -lm
   Source for libf2c is in /netlib/f2c/libf2c.zip, e.g.,

http://www.netlib.org/f2c/libf2c.zip
*/

/* Subroutine */ int sgemm_(char *transa, char *transb, integer *m, integer *
    n, integer *k, real *alpha, real *a, integer *lda, real *b, integer *
    ldb, real *beta, real *c__, integer *ldc, ftnlen transa_len, ftnlen 
    transb_len)
{
  /* System generated locals */
  integer a_dim1, a_offset, b_dim1, b_offset, c_dim1, c_offset, i__1, i__2, 
          i__3;

  /* Local variables */
  static integer i__, j, l, info;
  static logical nota, notb;
  static real temp;
  /*static integer ncola;*/
  extern logical lsame_(char *, char *, ftnlen, ftnlen);
  static integer nrowa, nrowb;

  (void)(transa_len);
  (void)(transb_len);

  /*     .. Scalar Arguments .. */
  /*     .. */
  /*     .. Array Arguments .. */
  /*     .. */

  /*  Purpose */
  /*  ======= */

  /*  SGEMM  performs one of the matrix-matrix operations */

  /*     C := alpha*op( A )*op( B ) + beta*C, */

  /*  where  op( X ) is one of */

  /*     op( X ) = X   or   op( X ) = X', */

  /*  alpha and beta are scalars, and A, B and C are matrices, with op( A ) */
  /*  an m by k matrix,  op( B )  a  k by n matrix and  C an m by n matrix. */

  /*  Arguments */
  /*  ========== */

  /*  TRANSA - CHARACTER*1. */
  /*           On entry, TRANSA specifies the form of op( A ) to be used in */
  /*           the matrix multiplication as follows: */

  /*              TRANSA = 'N' or 'n',  op( A ) = A. */

  /*              TRANSA = 'T' or 't',  op( A ) = A'. */

  /*              TRANSA = 'C' or 'c',  op( A ) = A'. */

  /*           Unchanged on exit. */

  /*  TRANSB - CHARACTER*1. */
  /*           On entry, TRANSB specifies the form of op( B ) to be used in */
  /*           the matrix multiplication as follows: */

  /*              TRANSB = 'N' or 'n',  op( B ) = B. */

  /*              TRANSB = 'T' or 't',  op( B ) = B'. */

  /*              TRANSB = 'C' or 'c',  op( B ) = B'. */

  /*           Unchanged on exit. */

  /*  M      - INTEGER. */
  /*           On entry,  M  specifies  the number  of rows  of the  matrix */
  /*           op( A )  and of the  matrix  C.  M  must  be at least  zero. */
  /*           Unchanged on exit. */

  /*  N      - INTEGER. */
  /*           On entry,  N  specifies the number  of columns of the matrix */
  /*           op( B ) and the number of columns of the matrix C. N must be */
  /*           at least zero. */
  /*           Unchanged on exit. */

  /*  K      - INTEGER. */
  /*           On entry,  K  specifies  the number of columns of the matrix */
  /*           op( A ) and the number of rows of the matrix op( B ). K must */
  /*           be at least  zero. */
  /*           Unchanged on exit. */

  /*  ALPHA  - REAL            . */
  /*           On entry, ALPHA specifies the scalar alpha. */
  /*           Unchanged on exit. */

  /*  A      - REAL             array of DIMENSION ( LDA, ka ), where ka is */
  /*           k  when  TRANSA = 'N' or 'n',  and is  m  otherwise. */
  /*           Before entry with  TRANSA = 'N' or 'n',  the leading  m by k */
  /*           part of the array  A  must contain the matrix  A,  otherwise */
  /*           the leading  k by m  part of the array  A  must contain  the */
  /*           matrix A. */
  /*           Unchanged on exit. */

  /*  LDA    - INTEGER. */
  /*           On entry, LDA specifies the first dimension of A as declared */
  /*           in the calling (sub) program. When  TRANSA = 'N' or 'n' then */
  /*           LDA must be at least  max( 1, m ), otherwise  LDA must be at */
  /*           least  max( 1, k ). */
  /*           Unchanged on exit. */

  /*  B      - REAL             array of DIMENSION ( LDB, kb ), where kb is */
  /*           n  when  TRANSB = 'N' or 'n',  and is  k  otherwise. */
  /*           Before entry with  TRANSB = 'N' or 'n',  the leading  k by n */
  /*           part of the array  B  must contain the matrix  B,  otherwise */
  /*           the leading  n by k  part of the array  B  must contain  the */
  /*           matrix B. */
  /*           Unchanged on exit. */

  /*  LDB    - INTEGER. */
  /*           On entry, LDB specifies the first dimension of B as declared */
  /*           in the calling (sub) program. When  TRANSB = 'N' or 'n' then */
  /*           LDB must be at least  max( 1, k ), otherwise  LDB must be at */
  /*           least  max( 1, n ). */
  /*           Unchanged on exit. */

  /*  BETA   - REAL            . */
  /*           On entry,  BETA  specifies the scalar  beta.  When  BETA  is */
  /*           supplied as zero then C need not be set on input. */
  /*           Unchanged on exit. */

  /*  C      - REAL             array of DIMENSION ( LDC, n ). */
  /*           Before entry, the leading  m by n  part of the array  C must */
  /*           contain the matrix  C,  except when  beta  is zero, in which */
  /*           case C need not be set on entry. */
  /*           On exit, the array  C  is overwritten by the  m by n  matrix */
  /*           ( alpha*op( A )*op( B ) + beta*C ). */

  /*  LDC    - INTEGER. */
  /*           On entry, LDC specifies the first dimension of C as declared */
  /*           in  the  calling  (sub)  program.   LDC  must  be  at  least */
  /*           max( 1, m ). */
  /*           Unchanged on exit. */

  /*  Further Details */
  /*  =============== */

  /*  Level 3 Blas routine. */

  /*  -- Written on 8-February-1989. */
  /*     Jack Dongarra, Argonne National Laboratory. */
  /*     Iain Duff, AERE Harwell. */
  /*     Jeremy Du Croz, Numerical Algorithms Group Ltd. */
  /*     Sven Hammarling, Numerical Algorithms Group Ltd. */

  /*  ===================================================================== */

  /*     .. External Functions .. */
  /*     .. */
  /*     .. External Subroutines .. */
  /*     .. */
  /*     .. Intrinsic Functions .. */
  /*     .. */
  /*     .. Local Scalars .. */
  /*     .. */
  /*     .. Parameters .. */
  /*     .. */

  /*     Set  NOTA  and  NOTB  as  true if  A  and  B  respectively are not */
  /*     transposed and set  NROWA, NCOLA and  NROWB  as the number of rows */
  /*     and  columns of  A  and the  number of  rows  of  B  respectively. */

  /* Parameter adjustments */
  a_dim1 = *lda;
  a_offset = 1 + a_dim1;
  a -= a_offset;
  b_dim1 = *ldb;
  b_offset = 1 + b_dim1;
  b -= b_offset;
  c_dim1 = *ldc;
  c_offset = 1 + c_dim1;
  c__ -= c_offset;

  /* Function Body */
  nota = lsame_(transa, "N", (ftnlen)1, (ftnlen)1);
  notb = lsame_(transb, "N", (ftnlen)1, (ftnlen)1);
  if (nota) {
    nrowa = *m;
    /*ncola = *k;*/
  } else {
    nrowa = *k;
    /*ncola = *m;*/
  }
  if (notb) {
    nrowb = *k;
  } else {
    nrowb = *n;
  }

  /*     Test the input parameters. */

  info = 0;
  if (! nota && ! lsame_(transa, "C", (ftnlen)1, (ftnlen)1) && ! lsame_(
        transa, "T", (ftnlen)1, (ftnlen)1)) {
    info = 1;
  } else if (! notb && ! lsame_(transb, "C", (ftnlen)1, (ftnlen)1) && ! 
      lsame_(transb, "T", (ftnlen)1, (ftnlen)1)) {
    info = 2;
  } else if (*m < 0) {
    info = 3;
  } else if (*n < 0) {
    info = 4;
  } else if (*k < 0) {
    info = 5;
  } else if (*lda < max(1,nrowa)) {
    info = 8;
  } else if (*ldb < max(1,nrowb)) {
    info = 10;
  } else if (*ldc < max(1,*m)) {
    info = 13;
  }
  if (info != 0) {
    return 0;
  }

  /*     Quick return if possible. */

  if (_dequal(*m, 0.0) || _dequal(*n, 0.0) || ((_dequal(*alpha, 0.0f) || _dequal(*k, 0.0)) && _dequal(*beta, 1.f))) 
  {
    return 0;
  }

  /*     And if  alpha.eq.zero. */

  if (_dequal(*alpha, 0.f)) {
    if (_dequal(*beta, 0.f)) {
      i__1 = *n;
      for (j = 1; j <= i__1; ++j) {
        i__2 = *m;
        for (i__ = 1; i__ <= i__2; ++i__) {
          c__[i__ + j * c_dim1] = 0.f;
          /* L10: */
        }
        /* L20: */
      }
    } else {
      i__1 = *n;
      for (j = 1; j <= i__1; ++j) {
        i__2 = *m;
        for (i__ = 1; i__ <= i__2; ++i__) {
          c__[i__ + j * c_dim1] = *beta * c__[i__ + j * c_dim1];
          /* L30: */
        }
        /* L40: */
      }
    }
    return 0;
  }

  /*     Start the operations. */

  if (notb) {
    if (nota) {
      /*           Form  C := alpha*A*B + beta*C. */
      i__1 = *n;
      for (j = 1; j <= i__1; ++j) {
        if (_dequal(*beta, 0.f)) {
          i__2 = *m;
          for (i__ = 1; i__ <= i__2; ++i__) {
            c__[i__ + j * c_dim1] = 0.f;
            /* L50: */
          }
        } else if (!_dequal(*beta, 1.f)) {
          i__2 = *m;
          for (i__ = 1; i__ <= i__2; ++i__) {
            c__[i__ + j * c_dim1] = *beta * c__[i__ + j * c_dim1];
            /* L60: */
          }
        }
        i__2 = *k;
        for (l = 1; l <= i__2; ++l) {
          if (!_dequal(b[l + j * b_dim1], 0.f)) {
            temp = *alpha * b[l + j * b_dim1];
            i__3 = *m;
            for (i__ = 1; i__ <= i__3; ++i__) {
              c__[i__ + j * c_dim1] += temp * a[i__ + l * 
                a_dim1];
              /* L70: */
            }
          }
          /* L80: */
        }
        /* L90: */
      }
    } else {

      /*           Form  C := alpha*A'*B + beta*C */

      i__1 = *n;
      for (j = 1; j <= i__1; ++j) {
        i__2 = *m;
        for (i__ = 1; i__ <= i__2; ++i__) {
          temp = 0.f;
          i__3 = *k;
          for (l = 1; l <= i__3; ++l) {
            temp += a[l + i__ * a_dim1] * b[l + j * b_dim1];
            /* L100: */
          }
          if (_dequal(*beta, 0.f)) {
            c__[i__ + j * c_dim1] = *alpha * temp;
          } else {
            c__[i__ + j * c_dim1] = *alpha * temp + *beta * c__[
              i__ + j * c_dim1];
          }
          /* L110: */
        }
        /* L120: */
      }
    }
  } else {
    if (nota) {

      /*           Form  C := alpha*A*B' + beta*C */

      i__1 = *n;
      for (j = 1; j <= i__1; ++j) {
        if (_dequal(*beta, 0.f)) {
          i__2 = *m;
          for (i__ = 1; i__ <= i__2; ++i__) {
            c__[i__ + j * c_dim1] = 0.f;
            /* L130: */
          }
        } else if (!_dequal(*beta, 1.f)) {
          i__2 = *m;
          for (i__ = 1; i__ <= i__2; ++i__) {
            c__[i__ + j * c_dim1] = *beta * c__[i__ + j * c_dim1];
            /* L140: */
          }
        }
        i__2 = *k;
        for (l = 1; l <= i__2; ++l) {
          if (!_dequal(b[j + l * b_dim1], 0.f)) {
            temp = *alpha * b[j + l * b_dim1];
            i__3 = *m;
            for (i__ = 1; i__ <= i__3; ++i__) {
              c__[i__ + j * c_dim1] += temp * a[i__ + l * 
                a_dim1];
              /* L150: */
            }
          }
          /* L160: */
        }
        /* L170: */
      }
    } else {

      /*           Form  C := alpha*A'*B' + beta*C */

      i__1 = *n;
      for (j = 1; j <= i__1; ++j) {
        i__2 = *m;
        for (i__ = 1; i__ <= i__2; ++i__) {
          temp = 0.f;
          i__3 = *k;
          for (l = 1; l <= i__3; ++l) {
            temp += a[l + i__ * a_dim1] * b[j + l * b_dim1];
            /* L180: */
          }
          if (_dequal(*beta, 0.f)) {
            c__[i__ + j * c_dim1] = *alpha * temp;
          } else {
            c__[i__ + j * c_dim1] = *alpha * temp + *beta * c__[
              i__ + j * c_dim1];
          }
          /* L190: */
        }
        /* L200: */
      }
    }
  }

  return 0;

  /*     End of SGEMM . */

} /* sgemm_ */

/* ../BLAS/SRC/sgemv.f -- translated by f2c (version 20090411).
   You must link the resulting object file with libf2c:
   on Microsoft Windows system, link with libf2c.lib;
   on Linux or Unix systems, link with .../path/to/libf2c.a -lm
   or, if you install libf2c.a in a standard place, with -lf2c -lm
   -- in that order, at the end of the command line, as in
   cc *.o -lf2c -lm
   Source for libf2c is in /netlib/f2c/libf2c.zip, e.g.,

http://www.netlib.org/f2c/libf2c.zip
*/

/* Subroutine */ int sgemv_(char *trans, integer *m, integer *n, real *alpha, 
    real *a, integer *lda, real *x, integer *incx, real *beta, real *y, 
    integer *incy, ftnlen trans_len)
{
  /* System generated locals */
  integer a_dim1, a_offset, i__1, i__2;

  /* Local variables */
  static integer i__, j, ix, iy, jx, jy, kx, ky, info;
  static real temp;
  static integer lenx, leny;
  extern logical lsame_(char *, char *, ftnlen, ftnlen);

  (void)(trans_len);

  /*     .. Scalar Arguments .. */
  /*     .. */
  /*     .. Array Arguments .. */
  /*     .. */

  /*  Purpose */
  /*  ======= */

  /*  SGEMV  performs one of the matrix-vector operations */

  /*     y := alpha*A*x + beta*y,   or   y := alpha*A'*x + beta*y, */

  /*  where alpha and beta are scalars, x and y are vectors and A is an */
  /*  m by n matrix. */

  /*  Arguments */
  /*  ========== */

  /*  TRANS  - CHARACTER*1. */
  /*           On entry, TRANS specifies the operation to be performed as */
  /*           follows: */

  /*              TRANS = 'N' or 'n'   y := alpha*A*x + beta*y. */

  /*              TRANS = 'T' or 't'   y := alpha*A'*x + beta*y. */

  /*              TRANS = 'C' or 'c'   y := alpha*A'*x + beta*y. */

  /*           Unchanged on exit. */

  /*  M      - INTEGER. */
  /*           On entry, M specifies the number of rows of the matrix A. */
  /*           M must be at least zero. */
  /*           Unchanged on exit. */

  /*  N      - INTEGER. */
  /*           On entry, N specifies the number of columns of the matrix A. */
  /*           N must be at least zero. */
  /*           Unchanged on exit. */

  /*  ALPHA  - REAL            . */
  /*           On entry, ALPHA specifies the scalar alpha. */
  /*           Unchanged on exit. */

  /*  A      - REAL             array of DIMENSION ( LDA, n ). */
  /*           Before entry, the leading m by n part of the array A must */
  /*           contain the matrix of coefficients. */
  /*           Unchanged on exit. */

  /*  LDA    - INTEGER. */
  /*           On entry, LDA specifies the first dimension of A as declared */
  /*           in the calling (sub) program. LDA must be at least */
  /*           max( 1, m ). */
  /*           Unchanged on exit. */

  /*  X      - REAL             array of DIMENSION at least */
  /*           ( 1 + ( n - 1 )*abs( INCX ) ) when TRANS = 'N' or 'n' */
  /*           and at least */
  /*           ( 1 + ( m - 1 )*abs( INCX ) ) otherwise. */
  /*           Before entry, the incremented array X must contain the */
  /*           vector x. */
  /*           Unchanged on exit. */

  /*  INCX   - INTEGER. */
  /*           On entry, INCX specifies the increment for the elements of */
  /*           X. INCX must not be zero. */
  /*           Unchanged on exit. */

  /*  BETA   - REAL            . */
  /*           On entry, BETA specifies the scalar beta. When BETA is */
  /*           supplied as zero then Y need not be set on input. */
  /*           Unchanged on exit. */

  /*  Y      - REAL             array of DIMENSION at least */
  /*           ( 1 + ( m - 1 )*abs( INCY ) ) when TRANS = 'N' or 'n' */
  /*           and at least */
  /*           ( 1 + ( n - 1 )*abs( INCY ) ) otherwise. */
  /*           Before entry with BETA non-zero, the incremented array Y */
  /*           must contain the vector y. On exit, Y is overwritten by the */
  /*           updated vector y. */

  /*  INCY   - INTEGER. */
  /*           On entry, INCY specifies the increment for the elements of */
  /*           Y. INCY must not be zero. */
  /*           Unchanged on exit. */

  /*  Further Details */
  /*  =============== */

  /*  Level 2 Blas routine. */

  /*  -- Written on 22-October-1986. */
  /*     Jack Dongarra, Argonne National Lab. */
  /*     Jeremy Du Croz, Nag Central Office. */
  /*     Sven Hammarling, Nag Central Office. */
  /*     Richard Hanson, Sandia National Labs. */

  /*  ===================================================================== */

  /*     .. Parameters .. */
  /*     .. */
  /*     .. Local Scalars .. */
  /*     .. */
  /*     .. External Functions .. */
  /*     .. */
  /*     .. External Subroutines .. */
  /*     .. */
  /*     .. Intrinsic Functions .. */
  /*     .. */

  /*     Test the input parameters. */

  /* Parameter adjustments */
  a_dim1 = *lda;
  a_offset = 1 + a_dim1;
  a -= a_offset;
  --x;
  --y;

  /* Function Body */
  info = 0;
  if (! lsame_(trans, "N", (ftnlen)1, (ftnlen)1) && ! lsame_(trans, "T", (
          ftnlen)1, (ftnlen)1) && ! lsame_(trans, "C", (ftnlen)1, (ftnlen)1)
     ) {
    info = 1;
  } else if (*m < 0) {
    info = 2;
  } else if (*n < 0) {
    info = 3;
  } else if (*lda < max(1,*m)) {
    info = 6;
  } else if (*incx == 0) {
    info = 8;
  } else if (*incy == 0) {
    info = 11;
  }
  if (info != 0) {
    return 0;
  }

  /*     Quick return if possible. */

  if (_dequal(*m, 0.0) || _dequal(*n, 0.0) || (_dequal(*alpha, 0.f) && _dequal(*beta, 1.f))) {
    return 0;
  }

  /*     Set  LENX  and  LENY, the lengths of the vectors x and y, and set */
  /*     up the start points in  X  and  Y. */

  if (lsame_(trans, "N", (ftnlen)1, (ftnlen)1)) {
    lenx = *n;
    leny = *m;
  } else {
    lenx = *m;
    leny = *n;
  }
  if (*incx > 0) {
    kx = 1;
  } else {
    kx = 1 - (lenx - 1) * *incx;
  }
  if (*incy > 0) {
    ky = 1;
  } else {
    ky = 1 - (leny - 1) * *incy;
  }

  /*     Start the operations. In this version the elements of A are */
  /*     accessed sequentially with one pass through A. */

  /*     First form  y := beta*y. */

  if (!_dequal(*beta, 1.f)) {
    if (*incy == 1) {
      if (_dequal(*beta, 0.f)) {
        i__1 = leny;
        for (i__ = 1; i__ <= i__1; ++i__) {
          y[i__] = 0.f;
          /* L10: */
        }
      } else {
        i__1 = leny;
        for (i__ = 1; i__ <= i__1; ++i__) {
          y[i__] = *beta * y[i__];
          /* L20: */
        }
      }
    } else {
      iy = ky;
      if (_dequal(*beta, 0.f)) {
        i__1 = leny;
        for (i__ = 1; i__ <= i__1; ++i__) {
          y[iy] = 0.f;
          iy += *incy;
          /* L30: */
        }
      } else {
        i__1 = leny;
        for (i__ = 1; i__ <= i__1; ++i__) {
          y[iy] = *beta * y[iy];
          iy += *incy;
          /* L40: */
        }
      }
    }
  }
  if (_dequal(*alpha, 0.f)) {
    return 0;
  }
  if (lsame_(trans, "N", (ftnlen)1, (ftnlen)1)) {

    /*        Form  y := alpha*A*x + y. */

    jx = kx;
    if (*incy == 1) {
      i__1 = *n;
      for (j = 1; j <= i__1; ++j) {
        if (!_dequal(x[jx], 0.f)) {
          temp = *alpha * x[jx];
          i__2 = *m;
          for (i__ = 1; i__ <= i__2; ++i__) {
            y[i__] += temp * a[i__ + j * a_dim1];
            /* L50: */
          }
        }
        jx += *incx;
        /* L60: */
      }
    } else {
      i__1 = *n;
      for (j = 1; j <= i__1; ++j) {
        if (!_dequal(x[jx], 0.f)) {
          temp = *alpha * x[jx];
          iy = ky;
          i__2 = *m;
          for (i__ = 1; i__ <= i__2; ++i__) {
            y[iy] += temp * a[i__ + j * a_dim1];
            iy += *incy;
            /* L70: */
          }
        }
        jx += *incx;
        /* L80: */
      }
    }
  } else {

    /*        Form  y := alpha*A'*x + y. */

    jy = ky;
    if (*incx == 1) {
      i__1 = *n;
      for (j = 1; j <= i__1; ++j) {
        temp = 0.f;
        i__2 = *m;
        for (i__ = 1; i__ <= i__2; ++i__) {
          temp += a[i__ + j * a_dim1] * x[i__];
          /* L90: */
        }
        y[jy] += *alpha * temp;
        jy += *incy;
        /* L100: */
      }
    } else {
      i__1 = *n;
      for (j = 1; j <= i__1; ++j) {
        temp = 0.f;
        ix = kx;
        i__2 = *m;
        for (i__ = 1; i__ <= i__2; ++i__) {
          temp += a[i__ + j * a_dim1] * x[ix];
          ix += *incx;
          /* L110: */
        }
        y[jy] += *alpha * temp;
        jy += *incy;
        /* L120: */
      }
    }
  }

  return 0;

  /*     End of SGEMV . */

} /* sgemv_ */

/* Subroutine */ int sger_(integer *m, integer *n, real *alpha, real *x, 
    integer *incx, real *y, integer *incy, real *a, integer *lda)
{
  /* System generated locals */
  integer a_dim1, a_offset, i__1, i__2;

  /* Local variables */
  static integer i__, j, ix, jy, kx, info;
  static real temp;

  /*     .. Scalar Arguments .. */
  /*     .. */
  /*     .. Array Arguments .. */
  /*     .. */

  /*  Purpose */
  /*  ======= */

  /*  SGER   performs the rank 1 operation */

  /*     A := alpha*x*y' + A, */

  /*  where alpha is a scalar, x is an m element vector, y is an n element */
  /*  vector and A is an m by n matrix. */

  /*  Arguments */
  /*  ========== */

  /*  M      - INTEGER. */
  /*           On entry, M specifies the number of rows of the matrix A. */
  /*           M must be at least zero. */
  /*           Unchanged on exit. */

  /*  N      - INTEGER. */
  /*           On entry, N specifies the number of columns of the matrix A. */
  /*           N must be at least zero. */
  /*           Unchanged on exit. */

  /*  ALPHA  - REAL            . */
  /*           On entry, ALPHA specifies the scalar alpha. */
  /*           Unchanged on exit. */

  /*  X      - REAL             array of dimension at least */
  /*           ( 1 + ( m - 1 )*abs( INCX ) ). */
  /*           Before entry, the incremented array X must contain the m */
  /*           element vector x. */
  /*           Unchanged on exit. */

  /*  INCX   - INTEGER. */
  /*           On entry, INCX specifies the increment for the elements of */
  /*           X. INCX must not be zero. */
  /*           Unchanged on exit. */

  /*  Y      - REAL             array of dimension at least */
  /*           ( 1 + ( n - 1 )*abs( INCY ) ). */
  /*           Before entry, the incremented array Y must contain the n */
  /*           element vector y. */
  /*           Unchanged on exit. */

  /*  INCY   - INTEGER. */
  /*           On entry, INCY specifies the increment for the elements of */
  /*           Y. INCY must not be zero. */
  /*           Unchanged on exit. */

  /*  A      - REAL             array of DIMENSION ( LDA, n ). */
  /*           Before entry, the leading m by n part of the array A must */
  /*           contain the matrix of coefficients. On exit, A is */
  /*           overwritten by the updated matrix. */

  /*  LDA    - INTEGER. */
  /*           On entry, LDA specifies the first dimension of A as declared */
  /*           in the calling (sub) program. LDA must be at least */
  /*           max( 1, m ). */
  /*           Unchanged on exit. */

  /*  Further Details */
  /*  =============== */

  /*  Level 2 Blas routine. */

  /*  -- Written on 22-October-1986. */
  /*     Jack Dongarra, Argonne National Lab. */
  /*     Jeremy Du Croz, Nag Central Office. */
  /*     Sven Hammarling, Nag Central Office. */
  /*     Richard Hanson, Sandia National Labs. */

  /*  ===================================================================== */

  /*     .. Parameters .. */
  /*     .. */
  /*     .. Local Scalars .. */
  /*     .. */
  /*     .. External Subroutines .. */
  /*     .. */
  /*     .. Intrinsic Functions .. */
  /*     .. */

  /*     Test the input parameters. */

  /* Parameter adjustments */
  --x;
  --y;
  a_dim1 = *lda;
  a_offset = 1 + a_dim1;
  a -= a_offset;

  /* Function Body */
  info = 0;
  if (*m < 0) {
    info = 1;
  } else if (*n < 0) {
    info = 2;
  } else if (*incx == 0) {
    info = 5;
  } else if (*incy == 0) {
    info = 7;
  } else if (*lda < max(1,*m)) {
    info = 9;
  }
  if (info != 0) {
    return 0;
  }

  /*     Quick return if possible. */

  if (_dequal(*m, 0.0) || _dequal(*n, 0.0) || _dequal(*alpha, 0.f)) {
    return 0;
  }

  /*     Start the operations. In this version the elements of A are */
  /*     accessed sequentially with one pass through A. */

  if (*incy > 0) {
    jy = 1;
  } else {
    jy = 1 - (*n - 1) * *incy;
  }
  if (*incx == 1) {
    i__1 = *n;
    for (j = 1; j <= i__1; ++j) {
      if (!_dequal(y[jy], 0.f)) {
        temp = *alpha * y[jy];
        i__2 = *m;
        for (i__ = 1; i__ <= i__2; ++i__) {
          a[i__ + j * a_dim1] += x[i__] * temp;
          /* L10: */
        }
      }
      jy += *incy;
      /* L20: */
    }
  } else {
    if (*incx > 0) {
      kx = 1;
    } else {
      kx = 1 - (*m - 1) * *incx;
    }
    i__1 = *n;
    for (j = 1; j <= i__1; ++j) {
      if (!_dequal(y[jy], 0.f)) {
        temp = *alpha * y[jy];
        ix = kx;
        i__2 = *m;
        for (i__ = 1; i__ <= i__2; ++i__) {
          a[i__ + j * a_dim1] += x[ix] * temp;
          ix += *incx;
          /* L30: */
        }
      }
      jy += *incy;
      /* L40: */
    }
  }

  return 0;

  /*     End of SGER  . */

} /* sger_ */


/* sisnan.f -- translated by f2c (version 20090411).
   You must link the resulting object file with libf2c:
   on Microsoft Windows system, link with libf2c.lib;
   on Linux or Unix systems, link with .../path/to/libf2c.a -lm
   or, if you install libf2c.a in a standard place, with -lf2c -lm
   -- in that order, at the end of the command line, as in
   cc *.o -lf2c -lm
   Source for libf2c is in /netlib/f2c/libf2c.zip, e.g.,

http://www.netlib.org/f2c/libf2c.zip
*/

logical sisnan_(real *sin__)
{
  /* System generated locals */
  logical ret_val;

  /* Local variables */
  extern logical slaisnan_(real *, real *);


  /*  -- LAPACK auxiliary routine (version 3.2) -- */
  /*  -- LAPACK is a software package provided by Univ. of Tennessee,    -- */
  /*  -- Univ. of California Berkeley, Univ. of Colorado Denver and NAG Ltd..-- */
  /*     November 2006 */

  /*     .. Scalar Arguments .. */
  /*     .. */

  /*  Purpose */
  /*  ======= */

  /*  SISNAN returns .TRUE. if its argument is NaN, and .FALSE. */
  /*  otherwise.  To be replaced by the Fortran 2003 intrinsic in the */
  /*  future. */

  /*  Arguments */
  /*  ========= */

  /*  SIN      (input) REAL */
  /*          Input to test for NaN. */

  /*  ===================================================================== */

  /*  .. External Functions .. */
  /*  .. */
  /*  .. Executable Statements .. */
  ret_val = slaisnan_(sin__, sin__);
  return ret_val;
} /* sisnan_ */


logical slaisnan_(real *sin1, real *sin2)
{
  /* System generated locals */
  logical ret_val;


  /*  -- LAPACK auxiliary routine (version 3.2) -- */
  /*  -- LAPACK is a software package provided by Univ. of Tennessee,    -- */
  /*  -- Univ. of California Berkeley, Univ. of Colorado Denver and NAG Ltd..-- */
  /*     November 2006 */

  /*     .. Scalar Arguments .. */
  /*     .. */

  /*  Purpose */
  /*  ======= */

  /*  This routine is not for general use.  It exists solely to avoid */
  /*  over-optimization in SISNAN. */

  /*  SLAISNAN checks for NaNs by comparing its two arguments for */
  /*  inequality.  NaN is the only floating-point value where NaN != NaN */
  /*  returns .TRUE.  To check for NaNs, pass the same variable as both */
  /*  arguments. */

  /*  A compiler must assume that the two arguments are */
  /*  not the same variable, and the test will not be optimized away. */
  /*  Interprocedural or whole-program optimization may delete this */
  /*  test.  The ISNAN functions will be replaced by the correct */
  /*  Fortran 03 intrinsic once the intrinsic is widely available. */

  /*  Arguments */
  /*  ========= */

  /*  SIN1     (input) REAL */
  /*  SIN2     (input) REAL */
  /*          Two numbers to compare for inequality. */

  /*  ===================================================================== */

  /*  .. Executable Statements .. */
  ret_val = !_dequal(*sin1, *sin2);
  return ret_val;
} /* slaisnan_ */

/* slasyf.f -- translated by f2c (version 20090411).
   You must link the resulting object file with libf2c:
   on Microsoft Windows system, link with libf2c.lib;
   on Linux or Unix systems, link with .../path/to/libf2c.a -lm
   or, if you install libf2c.a in a standard place, with -lf2c -lm
   -- in that order, at the end of the command line, as in
   cc *.o -lf2c -lm
   Source for libf2c is in /netlib/f2c/libf2c.zip, e.g.,

http://www.netlib.org/f2c/libf2c.zip
*/

/* Table of constant values */

static real c_b8 = -1.f;
static real c_b9 = 1.f;

/* Subroutine */ int slasyf_(char *uplo, integer *n, integer *nb, integer *kb,
    real *a, integer *lda, integer *ipiv, real *w, integer *ldw, integer 
    *info, ftnlen uplo_len)
{
  /* System generated locals */
  integer a_dim1, a_offset, w_dim1, w_offset, i__1, i__2, i__3, i__4, i__5;
  real r__1, r__2, r__3;

  /* Builtin functions */
  double sqrt(doublereal);

  /* Local variables */
  static integer j, k;
  static real t, r1, d11, d21, d22;
  static integer jb, jj, kk, jp, kp, kw, kkw, imax, jmax;
  static real alpha;
  extern logical lsame_(char *, char *, ftnlen, ftnlen);
  extern /* Subroutine */ int sscal_(integer *, real *, real *, integer *), 
         sgemm_(char *, char *, integer *, integer *, integer *, real *, 
             real *, integer *, real *, integer *, real *, real *, integer *, 
             ftnlen, ftnlen), sgemv_(char *, integer *, integer *, real *, 
               real *, integer *, real *, integer *, real *, real *, integer *, 
               ftnlen);
  static integer kstep;
  extern /* Subroutine */ int scopy_(integer *, real *, integer *, real *, 
      integer *), sswap_(integer *, real *, integer *, real *, integer *
        );
  static real absakk;
  extern integer isamax_(integer *, real *, integer *);
  static real colmax, rowmax;

  (void)(uplo_len);

  /*  -- LAPACK routine (version 3.2) -- */
  /*  -- LAPACK is a software package provided by Univ. of Tennessee,    -- */
  /*  -- Univ. of California Berkeley, Univ. of Colorado Denver and NAG Ltd..-- */
  /*     November 2006 */

  /*     .. Scalar Arguments .. */
  /*     .. */
  /*     .. Array Arguments .. */
  /*     .. */

  /*  Purpose */
  /*  ======= */

  /*  SLASYF computes a partial factorization of a real symmetric matrix A */
  /*  using the Bunch-Kaufman diagonal pivoting method. The partial */
  /*  factorization has the form: */

  /*  A  =  ( I  U12 ) ( A11  0  ) (  I    0   )  if UPLO = 'U', or: */
  /*        ( 0  U22 ) (  0   D  ) ( U12' U22' ) */

  /*  A  =  ( L11  0 ) (  D   0  ) ( L11' L21' )  if UPLO = 'L' */
  /*        ( L21  I ) (  0  A22 ) (  0    I   ) */

  /*  where the order of D is at most NB. The actual order is returned in */
  /*  the argument KB, and is either NB or NB-1, or N if N <= NB. */

  /*  SLASYF is an auxiliary routine called by SSYTRF. It uses blocked code */
  /*  (calling Level 3 BLAS) to update the submatrix A11 (if UPLO = 'U') or */
  /*  A22 (if UPLO = 'L'). */

  /*  Arguments */
  /*  ========= */

  /*  UPLO    (input) CHARACTER*1 */
  /*          Specifies whether the upper or lower triangular part of the */
  /*          symmetric matrix A is stored: */
  /*          = 'U':  Upper triangular */
  /*          = 'L':  Lower triangular */

  /*  N       (input) INTEGER */
  /*          The order of the matrix A.  N >= 0. */

  /*  NB      (input) INTEGER */
  /*          The maximum number of columns of the matrix A that should be */
  /*          factored.  NB should be at least 2 to allow for 2-by-2 pivot */
  /*          blocks. */

  /*  KB      (output) INTEGER */
  /*          The number of columns of A that were actually factored. */
  /*          KB is either NB-1 or NB, or N if N <= NB. */

  /*  A       (input/output) REAL array, dimension (LDA,N) */
  /*          On entry, the symmetric matrix A.  If UPLO = 'U', the leading */
  /*          n-by-n upper triangular part of A contains the upper */
  /*          triangular part of the matrix A, and the strictly lower */
  /*          triangular part of A is not referenced.  If UPLO = 'L', the */
  /*          leading n-by-n lower triangular part of A contains the lower */
  /*          triangular part of the matrix A, and the strictly upper */
  /*          triangular part of A is not referenced. */
  /*          On exit, A contains details of the partial factorization. */

  /*  LDA     (input) INTEGER */
  /*          The leading dimension of the array A.  LDA >= max(1,N). */

  /*  IPIV    (output) INTEGER array, dimension (N) */
  /*          Details of the interchanges and the block structure of D. */
  /*          If UPLO = 'U', only the last KB elements of IPIV are set; */
  /*          if UPLO = 'L', only the first KB elements are set. */

  /*          If IPIV(k) > 0, then rows and columns k and IPIV(k) were */
  /*          interchanged and D(k,k) is a 1-by-1 diagonal block. */
  /*          If UPLO = 'U' and IPIV(k) = IPIV(k-1) < 0, then rows and */
  /*          columns k-1 and -IPIV(k) were interchanged and D(k-1:k,k-1:k) */
  /*          is a 2-by-2 diagonal block.  If UPLO = 'L' and IPIV(k) = */
  /*          IPIV(k+1) < 0, then rows and columns k+1 and -IPIV(k) were */
  /*          interchanged and D(k:k+1,k:k+1) is a 2-by-2 diagonal block. */

  /*  W       (workspace) REAL array, dimension (LDW,NB) */

  /*  LDW     (input) INTEGER */
  /*          The leading dimension of the array W.  LDW >= max(1,N). */

  /*  INFO    (output) INTEGER */
  /*          = 0: successful exit */
  /*          > 0: if INFO = k, D(k,k) is exactly zero.  The factorization */
  /*               has been completed, but the block diagonal matrix D is */
  /*               exactly singular. */

  /*  ===================================================================== */

  /*     .. Parameters .. */
  /*     .. */
  /*     .. Local Scalars .. */
  /*     .. */
  /*     .. External Functions .. */
  /*     .. */
  /*     .. External Subroutines .. */
  /*     .. */
  /*     .. Intrinsic Functions .. */
  /*     .. */
  /*     .. Executable Statements .. */

  /* Parameter adjustments */
  a_dim1 = *lda;
  a_offset = 1 + a_dim1;
  a -= a_offset;
  --ipiv;
  w_dim1 = *ldw;
  w_offset = 1 + w_dim1;
  w -= w_offset;

  /* Function Body */
  *info = 0;

  /*     Initialize ALPHA for use in choosing pivot block size. */

  alpha = (sqrt(17.f) + 1.f) / 8.f;

  if (lsame_(uplo, "U", (ftnlen)1, (ftnlen)1)) {

    /*        Factorize the trailing columns of A using the upper triangle */
    /*        of A and working backwards, and compute the matrix W = U12*D */
    /*        for use in updating A11 */

    /*        K is the main loop index, decreasing from N in steps of 1 or 2 */

    /*        KW is the column of W which corresponds to column K of A */

    k = *n;
L10:
    kw = *nb + k - *n;

    /*        Exit from loop */

    if ((k <= *n - *nb + 1 && *nb < *n) || k < 1) {
      goto L30;
    }

    /*        Copy column K of A to column KW of W and update it */

    scopy_(&k, &a[k * a_dim1 + 1], &c__1, &w[kw * w_dim1 + 1], &c__1);
    if (k < *n) {
      i__1 = *n - k;
      sgemv_("No transpose", &k, &i__1, &c_b8, &a[(k + 1) * a_dim1 + 1],
          lda, &w[k + (kw + 1) * w_dim1], ldw, &c_b9, &w[kw * 
          w_dim1 + 1], &c__1, (ftnlen)12);
    }

    kstep = 1;

    /*        Determine rows and columns to be interchanged and whether */
    /*        a 1-by-1 or 2-by-2 pivot block will be used */

    absakk = (r__1 = w[k + kw * w_dim1], dabs(r__1));

    /*        IMAX is the row-index of the largest off-diagonal element in */
    /*        column K, and COLMAX is its absolute value */

    if (k > 1) {
      i__1 = k - 1;
      imax = isamax_(&i__1, &w[kw * w_dim1 + 1], &c__1);
      colmax = (r__1 = w[imax + kw * w_dim1], dabs(r__1));
    } else {
      colmax = 0.f;
    }

    if (_dequal(dmax(absakk,colmax), 0.f)) {

      /*           Column K is zero: set INFO and continue */

      if (*info == 0) {
        *info = k;
      }
      kp = k;
    } else {
      if (absakk >= alpha * colmax) {

        /*              no interchange, use 1-by-1 pivot block */

        kp = k;
      } else {

        /*              Copy column IMAX to column KW-1 of W and update it */

        scopy_(&imax, &a[imax * a_dim1 + 1], &c__1, &w[(kw - 1) * 
            w_dim1 + 1], &c__1);
        i__1 = k - imax;
        scopy_(&i__1, &a[imax + (imax + 1) * a_dim1], lda, &w[imax + 
            1 + (kw - 1) * w_dim1], &c__1);
        if (k < *n) {
          i__1 = *n - k;
          sgemv_("No transpose", &k, &i__1, &c_b8, &a[(k + 1) * 
              a_dim1 + 1], lda, &w[imax + (kw + 1) * w_dim1], 
              ldw, &c_b9, &w[(kw - 1) * w_dim1 + 1], &c__1, (
                ftnlen)12);
        }

        /*              JMAX is the column-index of the largest off-diagonal */
        /*              element in row IMAX, and ROWMAX is its absolute value */

        i__1 = k - imax;
        jmax = imax + isamax_(&i__1, &w[imax + 1 + (kw - 1) * w_dim1],
            &c__1);
        rowmax = (r__1 = w[jmax + (kw - 1) * w_dim1], dabs(r__1));
        if (imax > 1) {
          i__1 = imax - 1;
          jmax = isamax_(&i__1, &w[(kw - 1) * w_dim1 + 1], &c__1);
          /* Computing MAX */
          r__2 = rowmax, r__3 = (r__1 = w[jmax + (kw - 1) * w_dim1],
              dabs(r__1));
          rowmax = dmax(r__2,r__3);
        }

        if (absakk >= alpha * colmax * (colmax / rowmax)) {

          /*                 no interchange, use 1-by-1 pivot block */

          kp = k;
        } else if ((r__1 = w[imax + (kw - 1) * w_dim1], dabs(r__1)) >=
            alpha * rowmax) {

          /*                 interchange rows and columns K and IMAX, use 1-by-1 */
          /*                 pivot block */

          kp = imax;

          /*                 copy column KW-1 of W to column KW */

          scopy_(&k, &w[(kw - 1) * w_dim1 + 1], &c__1, &w[kw * 
              w_dim1 + 1], &c__1);
        } else {

          /*                 interchange rows and columns K-1 and IMAX, use 2-by-2 */
          /*                 pivot block */

          kp = imax;
          kstep = 2;
        }
      }

      kk = k - kstep + 1;
      kkw = *nb + kk - *n;

      /*           Updated column KP is already stored in column KKW of W */

      if (kp != kk) {

        /*              Copy non-updated column KK to column KP */

        a[kp + k * a_dim1] = a[kk + k * a_dim1];
        i__1 = k - 1 - kp;
        scopy_(&i__1, &a[kp + 1 + kk * a_dim1], &c__1, &a[kp + (kp + 
              1) * a_dim1], lda);
        scopy_(&kp, &a[kk * a_dim1 + 1], &c__1, &a[kp * a_dim1 + 1], &
            c__1);

        /*              Interchange rows KK and KP in last KK columns of A and W */

        i__1 = *n - kk + 1;
        sswap_(&i__1, &a[kk + kk * a_dim1], lda, &a[kp + kk * a_dim1],
            lda);
        i__1 = *n - kk + 1;
        sswap_(&i__1, &w[kk + kkw * w_dim1], ldw, &w[kp + kkw * 
            w_dim1], ldw);
      }

      if (kstep == 1) {

        /*              1-by-1 pivot block D(k): column KW of W now holds */

        /*              W(k) = U(k)*D(k) */

        /*              where U(k) is the k-th column of U */

        /*              Store U(k) in column k of A */

        scopy_(&k, &w[kw * w_dim1 + 1], &c__1, &a[k * a_dim1 + 1], &
            c__1);
        r1 = 1.f / a[k + k * a_dim1];
        i__1 = k - 1;
        sscal_(&i__1, &r1, &a[k * a_dim1 + 1], &c__1);
      } else {

        /*              2-by-2 pivot block D(k): columns KW and KW-1 of W now */
        /*              hold */

        /*              ( W(k-1) W(k) ) = ( U(k-1) U(k) )*D(k) */

        /*              where U(k) and U(k-1) are the k-th and (k-1)-th columns */
        /*              of U */

        if (k > 2) {

          /*                 Store U(k) and U(k-1) in columns k and k-1 of A */

          d21 = w[k - 1 + kw * w_dim1];
          d11 = w[k + kw * w_dim1] / d21;
          d22 = w[k - 1 + (kw - 1) * w_dim1] / d21;
          t = 1.f / (d11 * d22 - 1.f);
          d21 = t / d21;
          i__1 = k - 2;
          for (j = 1; j <= i__1; ++j) {
            a[j + (k - 1) * a_dim1] = d21 * (d11 * w[j + (kw - 1) 
                * w_dim1] - w[j + kw * w_dim1]);
            a[j + k * a_dim1] = d21 * (d22 * w[j + kw * w_dim1] - 
                w[j + (kw - 1) * w_dim1]);
            /* L20: */
          }
        }

        /*              Copy D(k) to A */

        a[k - 1 + (k - 1) * a_dim1] = w[k - 1 + (kw - 1) * w_dim1];
        a[k - 1 + k * a_dim1] = w[k - 1 + kw * w_dim1];
        a[k + k * a_dim1] = w[k + kw * w_dim1];
      }
    }

    /*        Store details of the interchanges in IPIV */

    if (kstep == 1) {
      ipiv[k] = kp;
    } else {
      ipiv[k] = -kp;
      ipiv[k - 1] = -kp;
    }

    /*        Decrease K and return to the start of the main loop */

    k -= kstep;
    goto L10;

L30:

    /*        Update the upper triangle of A11 (= A(1:k,1:k)) as */

    /*        A11 := A11 - U12*D*U12' = A11 - U12*W' */

    /*        computing blocks of NB columns at a time */

    i__1 = -(*nb);
    for (j = (k - 1) / *nb * *nb + 1; i__1 < 0 ? j >= 1 : j <= 1; j += 
        i__1) {
      /* Computing MIN */
      i__2 = *nb, i__3 = k - j + 1;
      jb = min(i__2,i__3);

      /*           Update the upper triangle of the diagonal block */

      i__2 = j + jb - 1;
      for (jj = j; jj <= i__2; ++jj) {
        i__3 = jj - j + 1;
        i__4 = *n - k;
        sgemv_("No transpose", &i__3, &i__4, &c_b8, &a[j + (k + 1) * 
            a_dim1], lda, &w[jj + (kw + 1) * w_dim1], ldw, &c_b9, 
            &a[j + jj * a_dim1], &c__1, (ftnlen)12);
        /* L40: */
      }

      /*           Update the rectangular superdiagonal block */

      i__2 = j - 1;
      i__3 = *n - k;
      sgemm_("No transpose", "Transpose", &i__2, &jb, &i__3, &c_b8, &a[(
            k + 1) * a_dim1 + 1], lda, &w[j + (kw + 1) * w_dim1], ldw,
          &c_b9, &a[j * a_dim1 + 1], lda, (ftnlen)12, (ftnlen)9);
      /* L50: */
    }

    /*        Put U12 in standard form by partially undoing the interchanges */
    /*        in columns k+1:n */

    j = k + 1;
L60:
    jj = j;
    jp = ipiv[j];
    if (jp < 0) {
      jp = -jp;
      ++j;
    }
    ++j;
    if (jp != jj && j <= *n) {
      i__1 = *n - j + 1;
      sswap_(&i__1, &a[jp + j * a_dim1], lda, &a[jj + j * a_dim1], lda);
    }
    if (j <= *n) {
      goto L60;
    }

    /*        Set KB to the number of columns factorized */

    *kb = *n - k;

  } else {

    /*        Factorize the leading columns of A using the lower triangle */
    /*        of A and working forwards, and compute the matrix W = L21*D */
    /*        for use in updating A22 */

    /*        K is the main loop index, increasing from 1 in steps of 1 or 2 */

    k = 1;
L70:

    /*        Exit from loop */

    if ((k >= *nb && *nb < *n) || k > *n) {
      goto L90;
    }

    /*        Copy column K of A to column K of W and update it */

    i__1 = *n - k + 1;
    scopy_(&i__1, &a[k + k * a_dim1], &c__1, &w[k + k * w_dim1], &c__1);
    i__1 = *n - k + 1;
    i__2 = k - 1;
    sgemv_("No transpose", &i__1, &i__2, &c_b8, &a[k + a_dim1], lda, &w[k 
        + w_dim1], ldw, &c_b9, &w[k + k * w_dim1], &c__1, (ftnlen)12);

    kstep = 1;

    /*        Determine rows and columns to be interchanged and whether */
    /*        a 1-by-1 or 2-by-2 pivot block will be used */

    absakk = (r__1 = w[k + k * w_dim1], dabs(r__1));

    /*        IMAX is the row-index of the largest off-diagonal element in */
    /*        column K, and COLMAX is its absolute value */

    if (k < *n) {
      i__1 = *n - k;
      imax = k + isamax_(&i__1, &w[k + 1 + k * w_dim1], &c__1);
      colmax = (r__1 = w[imax + k * w_dim1], dabs(r__1));
    } else {
      colmax = 0.f;
    }

    if (_dequal(dmax(absakk,colmax), 0.f)) {

      /*           Column K is zero: set INFO and continue */

      if (*info == 0) {
        *info = k;
      }
      kp = k;
    } else {
      if (absakk >= alpha * colmax) {

        /*              no interchange, use 1-by-1 pivot block */

        kp = k;
      } else {

        /*              Copy column IMAX to column K+1 of W and update it */

        i__1 = imax - k;
        scopy_(&i__1, &a[imax + k * a_dim1], lda, &w[k + (k + 1) * 
            w_dim1], &c__1);
        i__1 = *n - imax + 1;
        scopy_(&i__1, &a[imax + imax * a_dim1], &c__1, &w[imax + (k + 
              1) * w_dim1], &c__1);
        i__1 = *n - k + 1;
        i__2 = k - 1;
        sgemv_("No transpose", &i__1, &i__2, &c_b8, &a[k + a_dim1], 
            lda, &w[imax + w_dim1], ldw, &c_b9, &w[k + (k + 1) * 
            w_dim1], &c__1, (ftnlen)12);

        /*              JMAX is the column-index of the largest off-diagonal */
        /*              element in row IMAX, and ROWMAX is its absolute value */

        i__1 = imax - k;
        jmax = k - 1 + isamax_(&i__1, &w[k + (k + 1) * w_dim1], &c__1)
          ;
        rowmax = (r__1 = w[jmax + (k + 1) * w_dim1], dabs(r__1));
        if (imax < *n) {
          i__1 = *n - imax;
          jmax = imax + isamax_(&i__1, &w[imax + 1 + (k + 1) * 
              w_dim1], &c__1);
          /* Computing MAX */
          r__2 = rowmax, r__3 = (r__1 = w[jmax + (k + 1) * w_dim1], 
              dabs(r__1));
          rowmax = dmax(r__2,r__3);
        }

        if (absakk >= alpha * colmax * (colmax / rowmax)) {

          /*                 no interchange, use 1-by-1 pivot block */

          kp = k;
        } else if ((r__1 = w[imax + (k + 1) * w_dim1], dabs(r__1)) >= 
            alpha * rowmax) {

          /*                 interchange rows and columns K and IMAX, use 1-by-1 */
          /*                 pivot block */

          kp = imax;

          /*                 copy column K+1 of W to column K */

          i__1 = *n - k + 1;
          scopy_(&i__1, &w[k + (k + 1) * w_dim1], &c__1, &w[k + k * 
              w_dim1], &c__1);
        } else {

          /*                 interchange rows and columns K+1 and IMAX, use 2-by-2 */
          /*                 pivot block */

          kp = imax;
          kstep = 2;
        }
      }

      kk = k + kstep - 1;

      /*           Updated column KP is already stored in column KK of W */

      if (kp != kk) {

        /*              Copy non-updated column KK to column KP */

        a[kp + k * a_dim1] = a[kk + k * a_dim1];
        i__1 = kp - k - 1;
        scopy_(&i__1, &a[k + 1 + kk * a_dim1], &c__1, &a[kp + (k + 1) 
            * a_dim1], lda);
        i__1 = *n - kp + 1;
        scopy_(&i__1, &a[kp + kk * a_dim1], &c__1, &a[kp + kp * 
            a_dim1], &c__1);

        /*              Interchange rows KK and KP in first KK columns of A and W */

        sswap_(&kk, &a[kk + a_dim1], lda, &a[kp + a_dim1], lda);
        sswap_(&kk, &w[kk + w_dim1], ldw, &w[kp + w_dim1], ldw);
      }

      if (kstep == 1) {

        /*              1-by-1 pivot block D(k): column k of W now holds */

        /*              W(k) = L(k)*D(k) */

        /*              where L(k) is the k-th column of L */

        /*              Store L(k) in column k of A */

        i__1 = *n - k + 1;
        scopy_(&i__1, &w[k + k * w_dim1], &c__1, &a[k + k * a_dim1], &
            c__1);
        if (k < *n) {
          r1 = 1.f / a[k + k * a_dim1];
          i__1 = *n - k;
          sscal_(&i__1, &r1, &a[k + 1 + k * a_dim1], &c__1);
        }
      } else {

        /*              2-by-2 pivot block D(k): columns k and k+1 of W now hold */

        /*              ( W(k) W(k+1) ) = ( L(k) L(k+1) )*D(k) */

        /*              where L(k) and L(k+1) are the k-th and (k+1)-th columns */
        /*              of L */

        if (k < *n - 1) {

          /*                 Store L(k) and L(k+1) in columns k and k+1 of A */

          d21 = w[k + 1 + k * w_dim1];
          d11 = w[k + 1 + (k + 1) * w_dim1] / d21;
          d22 = w[k + k * w_dim1] / d21;
          t = 1.f / (d11 * d22 - 1.f);
          d21 = t / d21;
          i__1 = *n;
          for (j = k + 2; j <= i__1; ++j) {
            a[j + k * a_dim1] = d21 * (d11 * w[j + k * w_dim1] - 
                w[j + (k + 1) * w_dim1]);
            a[j + (k + 1) * a_dim1] = d21 * (d22 * w[j + (k + 1) *
                w_dim1] - w[j + k * w_dim1]);
            /* L80: */
          }
        }

        /*              Copy D(k) to A */

        a[k + k * a_dim1] = w[k + k * w_dim1];
        a[k + 1 + k * a_dim1] = w[k + 1 + k * w_dim1];
        a[k + 1 + (k + 1) * a_dim1] = w[k + 1 + (k + 1) * w_dim1];
      }
    }

    /*        Store details of the interchanges in IPIV */

    if (kstep == 1) {
      ipiv[k] = kp;
    } else {
      ipiv[k] = -kp;
      ipiv[k + 1] = -kp;
    }

    /*        Increase K and return to the start of the main loop */

    k += kstep;
    goto L70;

L90:

    /*        Update the lower triangle of A22 (= A(k:n,k:n)) as */

    /*        A22 := A22 - L21*D*L21' = A22 - L21*W' */

    /*        computing blocks of NB columns at a time */

    i__1 = *n;
    i__2 = *nb;
    for (j = k; i__2 < 0 ? j >= i__1 : j <= i__1; j += i__2) {
      /* Computing MIN */
      i__3 = *nb, i__4 = *n - j + 1;
      jb = min(i__3,i__4);

      /*           Update the lower triangle of the diagonal block */

      i__3 = j + jb - 1;
      for (jj = j; jj <= i__3; ++jj) {
        i__4 = j + jb - jj;
        i__5 = k - 1;
        sgemv_("No transpose", &i__4, &i__5, &c_b8, &a[jj + a_dim1], 
            lda, &w[jj + w_dim1], ldw, &c_b9, &a[jj + jj * a_dim1]
            , &c__1, (ftnlen)12);
        /* L100: */
      }

      /*           Update the rectangular subdiagonal block */

      if (j + jb <= *n) {
        i__3 = *n - j - jb + 1;
        i__4 = k - 1;
        sgemm_("No transpose", "Transpose", &i__3, &jb, &i__4, &c_b8, 
            &a[j + jb + a_dim1], lda, &w[j + w_dim1], ldw, &c_b9, 
            &a[j + jb + j * a_dim1], lda, (ftnlen)12, (ftnlen)9);
      }
      /* L110: */
    }

    /*        Put L21 in standard form by partially undoing the interchanges */
    /*        in columns 1:k-1 */

    j = k - 1;
L120:
    jj = j;
    jp = ipiv[j];
    if (jp < 0) {
      jp = -jp;
      --j;
    }
    --j;
    if (jp != jj && j >= 1) {
      sswap_(&j, &a[jp + a_dim1], lda, &a[jj + a_dim1], lda);
    }
    if (j >= 1) {
      goto L120;
    }

    /*        Set KB to the number of columns factorized */

    *kb = k - 1;

  }
  return 0;

  /*     End of SLASYF */

} /* slasyf_ */

/* ../BLAS/SRC/sscal.f -- translated by f2c (version 20090411).
   You must link the resulting object file with libf2c:
   on Microsoft Windows system, link with libf2c.lib;
   on Linux or Unix systems, link with .../path/to/libf2c.a -lm
   or, if you install libf2c.a in a standard place, with -lf2c -lm
   -- in that order, at the end of the command line, as in
   cc *.o -lf2c -lm
   Source for libf2c is in /netlib/f2c/libf2c.zip, e.g.,

http://www.netlib.org/f2c/libf2c.zip
*/

/* Subroutine */ int sscal_(integer *n, real *sa, real *sx, integer *incx)
{
  /* System generated locals */
  integer i__1, i__2;

  /* Local variables */
  static integer i__, m, mp1, nincx;

  /*     .. Scalar Arguments .. */
  /*     .. */
  /*     .. Array Arguments .. */
  /*     .. */

  /*  Purpose */
  /*  ======= */

  /*     scales a vector by a constant. */
  /*     uses unrolled loops for increment equal to 1. */

  /*  Further Details */
  /*  =============== */

  /*     jack dongarra, linpack, 3/11/78. */
  /*     modified 3/93 to return if incx .le. 0. */
  /*     modified 12/3/93, array(1) declarations changed to array(*) */

  /*  ===================================================================== */

  /*     .. Local Scalars .. */
  /*     .. */
  /*     .. Intrinsic Functions .. */
  /*     .. */
  /* Parameter adjustments */
  --sx;

  /* Function Body */
  if (*n <= 0 || *incx <= 0) {
    return 0;
  }
  if (*incx == 1) {
    goto L20;
  }

  /*        code for increment not equal to 1 */

  nincx = *n * *incx;
  i__1 = nincx;
  i__2 = *incx;
  for (i__ = 1; i__2 < 0 ? i__ >= i__1 : i__ <= i__1; i__ += i__2) {
    sx[i__] = *sa * sx[i__];
    /* L10: */
  }
  return 0;

  /*        code for increment equal to 1 */


  /*        clean-up loop */

L20:
  m = *n % 5;
  if (m == 0) {
    goto L40;
  }
  i__2 = m;
  for (i__ = 1; i__ <= i__2; ++i__) {
    sx[i__] = *sa * sx[i__];
    /* L30: */
  }
  if (*n < 5) {
    return 0;
  }
L40:
  mp1 = m + 1;
  i__2 = *n;
  for (i__ = mp1; i__ <= i__2; i__ += 5) {
    sx[i__] = *sa * sx[i__];
    sx[i__ + 1] = *sa * sx[i__ + 1];
    sx[i__ + 2] = *sa * sx[i__ + 2];
    sx[i__ + 3] = *sa * sx[i__ + 3];
    sx[i__ + 4] = *sa * sx[i__ + 4];
    /* L50: */
  }
  return 0;
} /* sscal_ */

/* ../BLAS/SRC/sswap.f -- translated by f2c (version 20090411).
   You must link the resulting object file with libf2c:
   on Microsoft Windows system, link with libf2c.lib;
   on Linux or Unix systems, link with .../path/to/libf2c.a -lm
   or, if you install libf2c.a in a standard place, with -lf2c -lm
   -- in that order, at the end of the command line, as in
   cc *.o -lf2c -lm
   Source for libf2c is in /netlib/f2c/libf2c.zip, e.g.,

http://www.netlib.org/f2c/libf2c.zip
*/

/* Subroutine */ int sswap_(integer *n, real *sx, integer *incx, real *sy, 
    integer *incy)
{
  /* System generated locals */
  integer i__1;

  /* Local variables */
  static integer i__, m, ix, iy, mp1;
  static real stemp;

  /*     .. Scalar Arguments .. */
  /*     .. */
  /*     .. Array Arguments .. */
  /*     .. */

  /*  Purpose */
  /*  ======= */

  /*     interchanges two vectors. */
  /*     uses unrolled loops for increments equal to 1. */

  /*  Further Details */
  /*  =============== */

  /*     jack dongarra, linpack, 3/11/78. */
  /*     modified 12/3/93, array(1) declarations changed to array(*) */

  /*  ===================================================================== */

  /*     .. Local Scalars .. */
  /*     .. */
  /*     .. Intrinsic Functions .. */
  /*     .. */
  /* Parameter adjustments */
  --sy;
  --sx;

  /* Function Body */
  if (*n <= 0) {
    return 0;
  }
  if (*incx == 1 && *incy == 1) {
    goto L20;
  }

  /*       code for unequal increments or equal increments not equal */
  /*         to 1 */

  ix = 1;
  iy = 1;
  if (*incx < 0) {
    ix = (-(*n) + 1) * *incx + 1;
  }
  if (*incy < 0) {
    iy = (-(*n) + 1) * *incy + 1;
  }
  i__1 = *n;
  for (i__ = 1; i__ <= i__1; ++i__) {
    stemp = sx[ix];
    sx[ix] = sy[iy];
    sy[iy] = stemp;
    ix += *incx;
    iy += *incy;
    /* L10: */
  }
  return 0;

  /*       code for both increments equal to 1 */


  /*       clean-up loop */

L20:
  m = *n % 3;
  if (m == 0) {
    goto L40;
  }
  i__1 = m;
  for (i__ = 1; i__ <= i__1; ++i__) {
    stemp = sx[i__];
    sx[i__] = sy[i__];
    sy[i__] = stemp;
    /* L30: */
  }
  if (*n < 3) {
    return 0;
  }
L40:
  mp1 = m + 1;
  i__1 = *n;
  for (i__ = mp1; i__ <= i__1; i__ += 3) {
    stemp = sx[i__];
    sx[i__] = sy[i__];
    sy[i__] = stemp;
    stemp = sx[i__ + 1];
    sx[i__ + 1] = sy[i__ + 1];
    sy[i__ + 1] = stemp;
    stemp = sx[i__ + 2];
    sx[i__ + 2] = sy[i__ + 2];
    sy[i__ + 2] = stemp;
    /* L50: */
  }
  return 0;
} /* sswap_ */

/* ../BLAS/SRC/ssyr.f -- translated by f2c (version 20090411).
   You must link the resulting object file with libf2c:
   on Microsoft Windows system, link with libf2c.lib;
   on Linux or Unix systems, link with .../path/to/libf2c.a -lm
   or, if you install libf2c.a in a standard place, with -lf2c -lm
   -- in that order, at the end of the command line, as in
   cc *.o -lf2c -lm
   Source for libf2c is in /netlib/f2c/libf2c.zip, e.g.,

http://www.netlib.org/f2c/libf2c.zip
*/

/* Subroutine */ int ssyr_(char *uplo, integer *n, real *alpha, real *x, 
    integer *incx, real *a, integer *lda, ftnlen uplo_len)
{
  /* System generated locals */
  integer a_dim1, a_offset, i__1, i__2;

  /* Local variables */
  static integer i__, j, ix, jx, kx, info;
  static real temp;
  extern logical lsame_(char *, char *, ftnlen, ftnlen);

  (void)(uplo_len);

  /*     .. Scalar Arguments .. */
  /*     .. */
  /*     .. Array Arguments .. */
  /*     .. */

  /*  Purpose */
  /*  ======= */

  /*  SSYR   performs the symmetric rank 1 operation */

  /*     A := alpha*x*x' + A, */

  /*  where alpha is a real scalar, x is an n element vector and A is an */
  /*  n by n symmetric matrix. */

  /*  Arguments */
  /*  ========== */

  /*  UPLO   - CHARACTER*1. */
  /*           On entry, UPLO specifies whether the upper or lower */
  /*           triangular part of the array A is to be referenced as */
  /*           follows: */

  /*              UPLO = 'U' or 'u'   Only the upper triangular part of A */
  /*                                  is to be referenced. */

  /*              UPLO = 'L' or 'l'   Only the lower triangular part of A */
  /*                                  is to be referenced. */

  /*           Unchanged on exit. */

  /*  N      - INTEGER. */
  /*           On entry, N specifies the order of the matrix A. */
  /*           N must be at least zero. */
  /*           Unchanged on exit. */

  /*  ALPHA  - REAL            . */
  /*           On entry, ALPHA specifies the scalar alpha. */
  /*           Unchanged on exit. */

  /*  X      - REAL             array of dimension at least */
  /*           ( 1 + ( n - 1 )*abs( INCX ) ). */
  /*           Before entry, the incremented array X must contain the n */
  /*           element vector x. */
  /*           Unchanged on exit. */

  /*  INCX   - INTEGER. */
  /*           On entry, INCX specifies the increment for the elements of */
  /*           X. INCX must not be zero. */
  /*           Unchanged on exit. */

  /*  A      - REAL             array of DIMENSION ( LDA, n ). */
  /*           Before entry with  UPLO = 'U' or 'u', the leading n by n */
  /*           upper triangular part of the array A must contain the upper */
  /*           triangular part of the symmetric matrix and the strictly */
  /*           lower triangular part of A is not referenced. On exit, the */
  /*           upper triangular part of the array A is overwritten by the */
  /*           upper triangular part of the updated matrix. */
  /*           Before entry with UPLO = 'L' or 'l', the leading n by n */
  /*           lower triangular part of the array A must contain the lower */
  /*           triangular part of the symmetric matrix and the strictly */
  /*           upper triangular part of A is not referenced. On exit, the */
  /*           lower triangular part of the array A is overwritten by the */
  /*           lower triangular part of the updated matrix. */

  /*  LDA    - INTEGER. */
  /*           On entry, LDA specifies the first dimension of A as declared */
  /*           in the calling (sub) program. LDA must be at least */
  /*           max( 1, n ). */
  /*           Unchanged on exit. */

  /*  Further Details */
  /*  =============== */

  /*  Level 2 Blas routine. */

  /*  -- Written on 22-October-1986. */
  /*     Jack Dongarra, Argonne National Lab. */
  /*     Jeremy Du Croz, Nag Central Office. */
  /*     Sven Hammarling, Nag Central Office. */
  /*     Richard Hanson, Sandia National Labs. */

  /*  ===================================================================== */

  /*     .. Parameters .. */
  /*     .. */
  /*     .. Local Scalars .. */
  /*     .. */
  /*     .. External Functions .. */
  /*     .. */
  /*     .. External Subroutines .. */
  /*     .. */
  /*     .. Intrinsic Functions .. */
  /*     .. */

  /*     Test the input parameters. */

  /* Parameter adjustments */
  --x;
  a_dim1 = *lda;
  a_offset = 1 + a_dim1;
  a -= a_offset;

  /* Function Body */
  info = 0;
  if (! lsame_(uplo, "U", (ftnlen)1, (ftnlen)1) && ! lsame_(uplo, "L", (
          ftnlen)1, (ftnlen)1)) {
    info = 1;
  } else if (*n < 0) {
    info = 2;
  } else if (*incx == 0) {
    info = 5;
  } else if (*lda < max(1,*n)) {
    info = 7;
  }
  if (info != 0) {
    return 0;
  }

  /*     Quick return if possible. */

  if (_dequal(*n, 0.0) || _dequal(*alpha, 0.f)) {
    return 0;
  }

  /*     Set the start point in X if the increment is not unity. */

  if (*incx <= 0) {
    kx = 1 - (*n - 1) * *incx;
  } else if (*incx != 1) {
    kx = 1;
  }

  /*     Start the operations. In this version the elements of A are */
  /*     accessed sequentially with one pass through the triangular part */
  /*     of A. */

  if (lsame_(uplo, "U", (ftnlen)1, (ftnlen)1)) {

    /*        Form  A  when A is stored in upper triangle. */

    if (*incx == 1) {
      i__1 = *n;
      for (j = 1; j <= i__1; ++j) {
        if (!_dequal(x[j], 0.f)) {
          temp = *alpha * x[j];
          i__2 = j;
          for (i__ = 1; i__ <= i__2; ++i__) {
            a[i__ + j * a_dim1] += x[i__] * temp;
            /* L10: */
          }
        }
        /* L20: */
      }
    } else {
      jx = kx;
      i__1 = *n;
      for (j = 1; j <= i__1; ++j) {
        if (!_dequal(x[jx], 0.f)) {
          temp = *alpha * x[jx];
          ix = kx;
          i__2 = j;
          for (i__ = 1; i__ <= i__2; ++i__) {
            a[i__ + j * a_dim1] += x[ix] * temp;
            ix += *incx;
            /* L30: */
          }
        }
        jx += *incx;
        /* L40: */
      }
    }
  } else {

    /*        Form  A  when A is stored in lower triangle. */

    if (*incx == 1) {
      i__1 = *n;
      for (j = 1; j <= i__1; ++j) {
        if (!_dequal(x[j], 0.f)) {
          temp = *alpha * x[j];
          i__2 = *n;
          for (i__ = j; i__ <= i__2; ++i__) {
            a[i__ + j * a_dim1] += x[i__] * temp;
            /* L50: */
          }
        }
        /* L60: */
      }
    } else {
      jx = kx;
      i__1 = *n;
      for (j = 1; j <= i__1; ++j) {
        if (!_dequal(x[jx], 0.f)) {
          temp = *alpha * x[jx];
          ix = jx;
          i__2 = *n;
          for (i__ = j; i__ <= i__2; ++i__) {
            a[i__ + j * a_dim1] += x[ix] * temp;
            ix += *incx;
            /* L70: */
          }
        }
        jx += *incx;
        /* L80: */
      }
    }
  }

  return 0;

  /*     End of SSYR  . */

} /* ssyr_ */

/* ssysv.f -- translated by f2c (version 20090411).
   You must link the resulting object file with libf2c:
   on Microsoft Windows system, link with libf2c.lib;
   on Linux or Unix systems, link with .../path/to/libf2c.a -lm
   or, if you install libf2c.a in a standard place, with -lf2c -lm
   -- in that order, at the end of the command line, as in
   cc *.o -lf2c -lm
   Source for libf2c is in /netlib/f2c/libf2c.zip, e.g.,

http://www.netlib.org/f2c/libf2c.zip
*/

/* Table of constant values */

static integer c_n1 = -1;

/* Subroutine */ int ssysv_(char *uplo, integer *n, integer *nrhs, real *a, 
    integer *lda, integer *ipiv, real *b, integer *ldb, real *work, 
    integer *lwork, integer *info, ftnlen uplo_len)
{
  /* System generated locals */
  integer a_dim1, a_offset, b_dim1, b_offset;/*, i__1;*/

  /* Local variables */
  static integer nb;
  extern logical lsame_(char *, char *, ftnlen, ftnlen);
  extern integer ilaenv_(integer *, char *, char *, integer *, integer *, 
      integer *, integer *, ftnlen, ftnlen);
  static integer lwkopt;
  static logical lquery;
  extern /* Subroutine */ int ssytrf_(char *, integer *, real *, integer *, 
      integer *, real *, integer *, integer *, ftnlen), ssytrs_(char *, 
        integer *, integer *, real *, integer *, integer *, real *, 
        integer *, integer *, ftnlen);

  (void)(uplo_len);

  /*  -- LAPACK driver routine (version 3.2) -- */
  /*  -- LAPACK is a software package provided by Univ. of Tennessee,    -- */
  /*  -- Univ. of California Berkeley, Univ. of Colorado Denver and NAG Ltd..-- */
  /*     November 2006 */

  /*     .. Scalar Arguments .. */
  /*     .. */
  /*     .. Array Arguments .. */
  /*     .. */

  /*  Purpose */
  /*  ======= */

  /*  SSYSV computes the solution to a real system of linear equations */
  /*     A * X = B, */
  /*  where A is an N-by-N symmetric matrix and X and B are N-by-NRHS */
  /*  matrices. */

  /*  The diagonal pivoting method is used to factor A as */
  /*     A = U * D * U**T,  if UPLO = 'U', or */
  /*     A = L * D * L**T,  if UPLO = 'L', */
  /*  where U (or L) is a product of permutation and unit upper (lower) */
  /*  triangular matrices, and D is symmetric and block diagonal with */
  /*  1-by-1 and 2-by-2 diagonal blocks.  The factored form of A is then */
  /*  used to solve the system of equations A * X = B. */

  /*  Arguments */
  /*  ========= */

  /*  UPLO    (input) CHARACTER*1 */
  /*          = 'U':  Upper triangle of A is stored; */
  /*          = 'L':  Lower triangle of A is stored. */

  /*  N       (input) INTEGER */
  /*          The number of linear equations, i.e., the order of the */
  /*          matrix A.  N >= 0. */

  /*  NRHS    (input) INTEGER */
  /*          The number of right hand sides, i.e., the number of columns */
  /*          of the matrix B.  NRHS >= 0. */

  /*  A       (input/output) REAL array, dimension (LDA,N) */
  /*          On entry, the symmetric matrix A.  If UPLO = 'U', the leading */
  /*          N-by-N upper triangular part of A contains the upper */
  /*          triangular part of the matrix A, and the strictly lower */
  /*          triangular part of A is not referenced.  If UPLO = 'L', the */
  /*          leading N-by-N lower triangular part of A contains the lower */
  /*          triangular part of the matrix A, and the strictly upper */
  /*          triangular part of A is not referenced. */

  /*          On exit, if INFO = 0, the block diagonal matrix D and the */
  /*          multipliers used to obtain the factor U or L from the */
  /*          factorization A = U*D*U**T or A = L*D*L**T as computed by */
  /*          SSYTRF. */

  /*  LDA     (input) INTEGER */
  /*          The leading dimension of the array A.  LDA >= max(1,N). */

  /*  IPIV    (output) INTEGER array, dimension (N) */
  /*          Details of the interchanges and the block structure of D, as */
  /*          determined by SSYTRF.  If IPIV(k) > 0, then rows and columns */
  /*          k and IPIV(k) were interchanged, and D(k,k) is a 1-by-1 */
  /*          diagonal block.  If UPLO = 'U' and IPIV(k) = IPIV(k-1) < 0, */
  /*          then rows and columns k-1 and -IPIV(k) were interchanged and */
  /*          D(k-1:k,k-1:k) is a 2-by-2 diagonal block.  If UPLO = 'L' and */
  /*          IPIV(k) = IPIV(k+1) < 0, then rows and columns k+1 and */
  /*          -IPIV(k) were interchanged and D(k:k+1,k:k+1) is a 2-by-2 */
  /*          diagonal block. */

  /*  B       (input/output) REAL array, dimension (LDB,NRHS) */
  /*          On entry, the N-by-NRHS right hand side matrix B. */
  /*          On exit, if INFO = 0, the N-by-NRHS solution matrix X. */

  /*  LDB     (input) INTEGER */
  /*          The leading dimension of the array B.  LDB >= max(1,N). */

  /*  WORK    (workspace/output) REAL array, dimension (MAX(1,LWORK)) */
  /*          On exit, if INFO = 0, WORK(1) returns the optimal LWORK. */

  /*  LWORK   (input) INTEGER */
  /*          The length of WORK.  LWORK >= 1, and for best performance */
  /*          LWORK >= max(1,N*NB), where NB is the optimal blocksize for */
  /*          SSYTRF. */

  /*          If LWORK = -1, then a workspace query is assumed; the routine */
  /*          only calculates the optimal size of the WORK array, returns */
  /*          this value as the first entry of the WORK array, and no error */
  /*          message related to LWORK is issued by XERBLA. */

  /*  INFO    (output) INTEGER */
  /*          = 0: successful exit */
  /*          < 0: if INFO = -i, the i-th argument had an illegal value */
  /*          > 0: if INFO = i, D(i,i) is exactly zero.  The factorization */
  /*               has been completed, but the block diagonal matrix D is */
  /*               exactly singular, so the solution could not be computed. */

  /*  ===================================================================== */

  /*     .. Local Scalars .. */
  /*     .. */
  /*     .. External Functions .. */
  /*     .. */
  /*     .. External Subroutines .. */
  /*     .. */
  /*     .. Intrinsic Functions .. */
  /*     .. */
  /*     .. Executable Statements .. */

  /*     Test the input parameters. */

  /* Parameter adjustments */
  a_dim1 = *lda;
  a_offset = 1 + a_dim1;
  a -= a_offset;
  --ipiv;
  b_dim1 = *ldb;
  b_offset = 1 + b_dim1;
  b -= b_offset;
  --work;

  /* Function Body */
  *info = 0;
  lquery = *lwork == -1;
  if (! lsame_(uplo, "U", (ftnlen)1, (ftnlen)1) && ! lsame_(uplo, "L", (
          ftnlen)1, (ftnlen)1)) {
    *info = -1;
  } else if (*n < 0) {
    *info = -2;
  } else if (*nrhs < 0) {
    *info = -3;
  } else if (*lda < max(1,*n)) {
    *info = -5;
  } else if (*ldb < max(1,*n)) {
    *info = -8;
  } else if (*lwork < 1 && ! lquery) {
    *info = -10;
  }

  if (*info == 0) {
    if (*n == 0) {
      lwkopt = 1;
    } else {
      nb = ilaenv_(&c__1, "SSYTRF", uplo, n, &c_n1, &c_n1, &c_n1, (
            ftnlen)6, (ftnlen)1);
      lwkopt = *n * nb;
    }
    work[1] = (real) lwkopt;
  }

  if (*info != 0) {
    /*i__1 = -(*info);*/
    return 0;
  } else if (lquery) {
    return 0;
  }

  /*     Compute the factorization A = U*D*U' or A = L*D*L'. */

  ssytrf_(uplo, n, &a[a_offset], lda, &ipiv[1], &work[1], lwork, info, (
        ftnlen)1);
  /*    if (*info == 0) {*/

  /*        Solve the system A*X = B, overwriting B with X. */

  ssytrs_(uplo, n, nrhs, &a[a_offset], lda, &ipiv[1], &b[b_offset], ldb,
      info, (ftnlen)1);

  /*    }*/

  work[1] = (real) lwkopt;

  return 0;

  /*     End of SSYSV */

} /* ssysv_ */

/* ssytf2.f -- translated by f2c (version 20090411).
   You must link the resulting object file with libf2c:
   on Microsoft Windows system, link with libf2c.lib;
   on Linux or Unix systems, link with .../path/to/libf2c.a -lm
   or, if you install libf2c.a in a standard place, with -lf2c -lm
   -- in that order, at the end of the command line, as in
   cc *.o -lf2c -lm
   Source for libf2c is in /netlib/f2c/libf2c.zip, e.g.,

http://www.netlib.org/f2c/libf2c.zip
*/

/* Subroutine */ int ssytf2_(char *uplo, integer *n, real *a, integer *lda, 
    integer *ipiv, integer *info, ftnlen uplo_len)
{
  /* System generated locals */
  integer a_dim1, a_offset, i__1, i__2;
  real r__1, r__2, r__3;

  /* Builtin functions */
  double sqrt(doublereal);

  /* Local variables */
  static integer i__, j, k;
  static real t, r1, d11, d12, d21, d22;
  static integer kk, kp;
  static real wk, wkm1, wkp1;
  static integer imax, jmax;
  extern /* Subroutine */ int ssyr_(char *, integer *, real *, real *, 
      integer *, real *, integer *, ftnlen);
  static real alpha;
  extern logical lsame_(char *, char *, ftnlen, ftnlen);
  extern /* Subroutine */ int sscal_(integer *, real *, real *, integer *);
  static integer kstep;
  static logical upper;
  extern /* Subroutine */ int sswap_(integer *, real *, integer *, real *, 
      integer *);
  static real absakk;
  extern integer isamax_(integer *, real *, integer *);
  static real colmax;
  extern logical sisnan_(real *);
  static real rowmax;


  (void)(uplo_len);
  /*  -- LAPACK routine (version 3.2) -- */
  /*  -- LAPACK is a software package provided by Univ. of Tennessee,    -- */
  /*  -- Univ. of California Berkeley, Univ. of Colorado Denver and NAG Ltd..-- */
  /*     November 2006 */

  /*     .. Scalar Arguments .. */
  /*     .. */
  /*     .. Array Arguments .. */
  /*     .. */

  /*  Purpose */
  /*  ======= */

  /*  SSYTF2 computes the factorization of a real symmetric matrix A using */
  /*  the Bunch-Kaufman diagonal pivoting method: */

  /*     A = U*D*U'  or  A = L*D*L' */

  /*  where U (or L) is a product of permutation and unit upper (lower) */
  /*  triangular matrices, U' is the transpose of U, and D is symmetric and */
  /*  block diagonal with 1-by-1 and 2-by-2 diagonal blocks. */

  /*  This is the unblocked version of the algorithm, calling Level 2 BLAS. */

  /*  Arguments */
  /*  ========= */

  /*  UPLO    (input) CHARACTER*1 */
  /*          Specifies whether the upper or lower triangular part of the */
  /*          symmetric matrix A is stored: */
  /*          = 'U':  Upper triangular */
  /*          = 'L':  Lower triangular */

  /*  N       (input) INTEGER */
  /*          The order of the matrix A.  N >= 0. */

  /*  A       (input/output) REAL array, dimension (LDA,N) */
  /*          On entry, the symmetric matrix A.  If UPLO = 'U', the leading */
  /*          n-by-n upper triangular part of A contains the upper */
  /*          triangular part of the matrix A, and the strictly lower */
  /*          triangular part of A is not referenced.  If UPLO = 'L', the */
  /*          leading n-by-n lower triangular part of A contains the lower */
  /*          triangular part of the matrix A, and the strictly upper */
  /*          triangular part of A is not referenced. */

  /*          On exit, the block diagonal matrix D and the multipliers used */
  /*          to obtain the factor U or L (see below for further details). */

  /*  LDA     (input) INTEGER */
  /*          The leading dimension of the array A.  LDA >= max(1,N). */

  /*  IPIV    (output) INTEGER array, dimension (N) */
  /*          Details of the interchanges and the block structure of D. */
  /*          If IPIV(k) > 0, then rows and columns k and IPIV(k) were */
  /*          interchanged and D(k,k) is a 1-by-1 diagonal block. */
  /*          If UPLO = 'U' and IPIV(k) = IPIV(k-1) < 0, then rows and */
  /*          columns k-1 and -IPIV(k) were interchanged and D(k-1:k,k-1:k) */
  /*          is a 2-by-2 diagonal block.  If UPLO = 'L' and IPIV(k) = */
  /*          IPIV(k+1) < 0, then rows and columns k+1 and -IPIV(k) were */
  /*          interchanged and D(k:k+1,k:k+1) is a 2-by-2 diagonal block. */

  /*  INFO    (output) INTEGER */
  /*          = 0: successful exit */
  /*          < 0: if INFO = -k, the k-th argument had an illegal value */
  /*          > 0: if INFO = k, D(k,k) is exactly zero.  The factorization */
  /*               has been completed, but the block diagonal matrix D is */
  /*               exactly singular, and division by zero will occur if it */
  /*               is used to solve a system of equations. */

  /*  Further Details */
  /*  =============== */

  /*  09-29-06 - patch from */
  /*    Bobby Cheng, MathWorks */

  /*    Replace l.204 and l.372 */
  /*         IF( MAX( ABSAKK, COLMAX ).EQ.ZERO ) THEN */
  /*    by */
  /*         IF( (MAX( ABSAKK, COLMAX ).EQ.ZERO) .OR. SISNAN(ABSAKK) ) THEN */

  /*  01-01-96 - Based on modifications by */
  /*    J. Lewis, Boeing Computer Services Company */
  /*    A. Petitet, Computer Science Dept., Univ. of Tenn., Knoxville, USA */
  /*  1-96 - Based on modifications by J. Lewis, Boeing Computer Services */
  /*         Company */

  /*  If UPLO = 'U', then A = U*D*U', where */
  /*     U = P(n)*U(n)* ... *P(k)U(k)* ..., */
  /*  i.e., U is a product of terms P(k)*U(k), where k decreases from n to */
  /*  1 in steps of 1 or 2, and D is a block diagonal matrix with 1-by-1 */
  /*  and 2-by-2 diagonal blocks D(k).  P(k) is a permutation matrix as */
  /*  defined by IPIV(k), and U(k) is a unit upper triangular matrix, such */
  /*  that if the diagonal block D(k) is of order s (s = 1 or 2), then */

  /*             (   I    v    0   )   k-s */
  /*     U(k) =  (   0    I    0   )   s */
  /*             (   0    0    I   )   n-k */
  /*                k-s   s   n-k */

  /*  If s = 1, D(k) overwrites A(k,k), and v overwrites A(1:k-1,k). */
  /*  If s = 2, the upper triangle of D(k) overwrites A(k-1,k-1), A(k-1,k), */
  /*  and A(k,k), and v overwrites A(1:k-2,k-1:k). */

  /*  If UPLO = 'L', then A = L*D*L', where */
  /*     L = P(1)*L(1)* ... *P(k)*L(k)* ..., */
  /*  i.e., L is a product of terms P(k)*L(k), where k increases from 1 to */
  /*  n in steps of 1 or 2, and D is a block diagonal matrix with 1-by-1 */
  /*  and 2-by-2 diagonal blocks D(k).  P(k) is a permutation matrix as */
  /*  defined by IPIV(k), and L(k) is a unit lower triangular matrix, such */
  /*  that if the diagonal block D(k) is of order s (s = 1 or 2), then */

  /*             (   I    0     0   )  k-1 */
  /*     L(k) =  (   0    I     0   )  s */
  /*             (   0    v     I   )  n-k-s+1 */
  /*                k-1   s  n-k-s+1 */

  /*  If s = 1, D(k) overwrites A(k,k), and v overwrites A(k+1:n,k). */
  /*  If s = 2, the lower triangle of D(k) overwrites A(k,k), A(k+1,k), */
  /*  and A(k+1,k+1), and v overwrites A(k+2:n,k:k+1). */

  /*  ===================================================================== */

  /*     .. Parameters .. */
  /*     .. */
  /*     .. Local Scalars .. */
  /*     .. */
  /*     .. External Functions .. */
  /*     .. */
  /*     .. External Subroutines .. */
  /*     .. */
  /*     .. Intrinsic Functions .. */
  /*     .. */
  /*     .. Executable Statements .. */

  /*     Test the input parameters. */

  /* Parameter adjustments */
  a_dim1 = *lda;
  a_offset = 1 + a_dim1;
  a -= a_offset;
  --ipiv;

  /* Function Body */
  *info = 0;
  upper = lsame_(uplo, "U", (ftnlen)1, (ftnlen)1);
  if (! upper && ! lsame_(uplo, "L", (ftnlen)1, (ftnlen)1)) {
    *info = -1;
  } else if (*n < 0) {
    *info = -2;
  } else if (*lda < max(1,*n)) {
    *info = -4;
  }
  if (*info != 0) {
    i__1 = -(*info);
    return 0;
  }

  /*     Initialize ALPHA for use in choosing pivot block size. */

  alpha = (sqrt(17.f) + 1.f) / 8.f;

  if (upper) {

    /*        Factorize A as U*D*U' using the upper triangle of A */

    /*        K is the main loop index, decreasing from N to 1 in steps of */
    /*        1 or 2 */

    k = *n;
L10:

    /*        If K < 1, exit from loop */

    if (k < 1) {
      goto L70;
    }
    kstep = 1;

    /*        Determine rows and columns to be interchanged and whether */
    /*        a 1-by-1 or 2-by-2 pivot block will be used */

    absakk = (r__1 = a[k + k * a_dim1], dabs(r__1));

    /*        IMAX is the row-index of the largest off-diagonal element in */
    /*        column K, and COLMAX is its absolute value */

    if (k > 1) {
      i__1 = k - 1;
      imax = isamax_(&i__1, &a[k * a_dim1 + 1], &c__1);
      colmax = (r__1 = a[imax + k * a_dim1], dabs(r__1));
    } else {
      colmax = 0.f;
    }

    if (_dequal(dmax(absakk,colmax), 0.f) || sisnan_(&absakk)) {

      /*           Column K is zero or contains a NaN: set INFO and continue */

      if (*info == 0) {
        *info = k;
      }
      kp = k;
    } else {
      if (absakk >= alpha * colmax) {

        /*              no interchange, use 1-by-1 pivot block */

        kp = k;
      } else {

        /*              JMAX is the column-index of the largest off-diagonal */
        /*              element in row IMAX, and ROWMAX is its absolute value */

        i__1 = k - imax;
        jmax = imax + isamax_(&i__1, &a[imax + (imax + 1) * a_dim1], 
            lda);
        rowmax = (r__1 = a[imax + jmax * a_dim1], dabs(r__1));
        if (imax > 1) {
          i__1 = imax - 1;
          jmax = isamax_(&i__1, &a[imax * a_dim1 + 1], &c__1);
          /* Computing MAX */
          r__2 = rowmax, r__3 = (r__1 = a[jmax + imax * a_dim1], 
              dabs(r__1));
          rowmax = dmax(r__2,r__3);
        }

        if (absakk >= alpha * colmax * (colmax / rowmax)) {

          /*                 no interchange, use 1-by-1 pivot block */

          kp = k;
        } else if ((r__1 = a[imax + imax * a_dim1], dabs(r__1)) >= 
            alpha * rowmax) {

          /*                 interchange rows and columns K and IMAX, use 1-by-1 */
          /*                 pivot block */

          kp = imax;
        } else {

          /*                 interchange rows and columns K-1 and IMAX, use 2-by-2 */
          /*                 pivot block */

          kp = imax;
          kstep = 2;
        }
      }

      kk = k - kstep + 1;
      if (kp != kk) {

        /*              Interchange rows and columns KK and KP in the leading */
        /*              submatrix A(1:k,1:k) */

        i__1 = kp - 1;
        sswap_(&i__1, &a[kk * a_dim1 + 1], &c__1, &a[kp * a_dim1 + 1],
            &c__1);
        i__1 = kk - kp - 1;
        sswap_(&i__1, &a[kp + 1 + kk * a_dim1], &c__1, &a[kp + (kp + 
              1) * a_dim1], lda);
        t = a[kk + kk * a_dim1];
        a[kk + kk * a_dim1] = a[kp + kp * a_dim1];
        a[kp + kp * a_dim1] = t;
        if (kstep == 2) {
          t = a[k - 1 + k * a_dim1];
          a[k - 1 + k * a_dim1] = a[kp + k * a_dim1];
          a[kp + k * a_dim1] = t;
        }
      }

      /*           Update the leading submatrix */

      if (kstep == 1) {

        /*              1-by-1 pivot block D(k): column k now holds */

        /*              W(k) = U(k)*D(k) */

        /*              where U(k) is the k-th column of U */

        /*              Perform a rank-1 update of A(1:k-1,1:k-1) as */

        /*              A := A - U(k)*D(k)*U(k)' = A - W(k)*1/D(k)*W(k)' */

        r1 = 1.f / a[k + k * a_dim1];
        i__1 = k - 1;
        r__1 = -r1;
        ssyr_(uplo, &i__1, &r__1, &a[k * a_dim1 + 1], &c__1, &a[
            a_offset], lda, (ftnlen)1);

        /*              Store U(k) in column k */

        i__1 = k - 1;
        sscal_(&i__1, &r1, &a[k * a_dim1 + 1], &c__1);
      } else {

        /*              2-by-2 pivot block D(k): columns k and k-1 now hold */

        /*              ( W(k-1) W(k) ) = ( U(k-1) U(k) )*D(k) */

        /*              where U(k) and U(k-1) are the k-th and (k-1)-th columns */
        /*              of U */

        /*              Perform a rank-2 update of A(1:k-2,1:k-2) as */

        /*              A := A - ( U(k-1) U(k) )*D(k)*( U(k-1) U(k) )' */
        /*                 = A - ( W(k-1) W(k) )*inv(D(k))*( W(k-1) W(k) )' */

        if (k > 2) {

          d12 = a[k - 1 + k * a_dim1];
          d22 = a[k - 1 + (k - 1) * a_dim1] / d12;
          d11 = a[k + k * a_dim1] / d12;
          t = 1.f / (d11 * d22 - 1.f);
          d12 = t / d12;

          for (j = k - 2; j >= 1; --j) {
            wkm1 = d12 * (d11 * a[j + (k - 1) * a_dim1] - a[j + k 
                * a_dim1]);
            wk = d12 * (d22 * a[j + k * a_dim1] - a[j + (k - 1) * 
                a_dim1]);
            for (i__ = j; i__ >= 1; --i__) {
              a[i__ + j * a_dim1] = a[i__ + j * a_dim1] - a[i__ 
                + k * a_dim1] * wk - a[i__ + (k - 1) * 
                a_dim1] * wkm1;
              /* L20: */
            }
            a[j + k * a_dim1] = wk;
            a[j + (k - 1) * a_dim1] = wkm1;
            /* L30: */
          }

        }

      }
    }

    /*        Store details of the interchanges in IPIV */

    if (kstep == 1) {
      ipiv[k] = kp;
    } else {
      ipiv[k] = -kp;
      ipiv[k - 1] = -kp;
    }

    /*        Decrease K and return to the start of the main loop */

    k -= kstep;
    goto L10;

  } else {

    /*        Factorize A as L*D*L' using the lower triangle of A */

    /*        K is the main loop index, increasing from 1 to N in steps of */
    /*        1 or 2 */

    k = 1;
L40:

    /*        If K > N, exit from loop */

    if (k > *n) {
      goto L70;
    }
    kstep = 1;

    /*        Determine rows and columns to be interchanged and whether */
    /*        a 1-by-1 or 2-by-2 pivot block will be used */

    absakk = (r__1 = a[k + k * a_dim1], dabs(r__1));

    /*        IMAX is the row-index of the largest off-diagonal element in */
    /*        column K, and COLMAX is its absolute value */

    if (k < *n) {
      i__1 = *n - k;
      imax = k + isamax_(&i__1, &a[k + 1 + k * a_dim1], &c__1);
      colmax = (r__1 = a[imax + k * a_dim1], dabs(r__1));
    } else {
      colmax = 0.f;
    }

    if (_dequal(dmax(absakk,colmax), 0.f) || sisnan_(&absakk)) {

      /*           Column K is zero or contains a NaN: set INFO and continue */

      if (*info == 0) {
        *info = k;
      }
      kp = k;
    } else {
      if (absakk >= alpha * colmax) {

        /*              no interchange, use 1-by-1 pivot block */

        kp = k;
      } else {

        /*              JMAX is the column-index of the largest off-diagonal */
        /*              element in row IMAX, and ROWMAX is its absolute value */

        i__1 = imax - k;
        jmax = k - 1 + isamax_(&i__1, &a[imax + k * a_dim1], lda);
        rowmax = (r__1 = a[imax + jmax * a_dim1], dabs(r__1));
        if (imax < *n) {
          i__1 = *n - imax;
          jmax = imax + isamax_(&i__1, &a[imax + 1 + imax * a_dim1],
              &c__1);
          /* Computing MAX */
          r__2 = rowmax, r__3 = (r__1 = a[jmax + imax * a_dim1], 
              dabs(r__1));
          rowmax = dmax(r__2,r__3);
        }

        if (absakk >= alpha * colmax * (colmax / rowmax)) {

          /*                 no interchange, use 1-by-1 pivot block */

          kp = k;
        } else if ((r__1 = a[imax + imax * a_dim1], dabs(r__1)) >= 
            alpha * rowmax) {

          /*                 interchange rows and columns K and IMAX, use 1-by-1 */
          /*                 pivot block */

          kp = imax;
        } else {

          /*                 interchange rows and columns K+1 and IMAX, use 2-by-2 */
          /*                 pivot block */

          kp = imax;
          kstep = 2;
        }
      }

      kk = k + kstep - 1;
      if (kp != kk) {

        /*              Interchange rows and columns KK and KP in the trailing */
        /*              submatrix A(k:n,k:n) */

        if (kp < *n) {
          i__1 = *n - kp;
          sswap_(&i__1, &a[kp + 1 + kk * a_dim1], &c__1, &a[kp + 1 
              + kp * a_dim1], &c__1);
        }
        i__1 = kp - kk - 1;
        sswap_(&i__1, &a[kk + 1 + kk * a_dim1], &c__1, &a[kp + (kk + 
              1) * a_dim1], lda);
        t = a[kk + kk * a_dim1];
        a[kk + kk * a_dim1] = a[kp + kp * a_dim1];
        a[kp + kp * a_dim1] = t;
        if (kstep == 2) {
          t = a[k + 1 + k * a_dim1];
          a[k + 1 + k * a_dim1] = a[kp + k * a_dim1];
          a[kp + k * a_dim1] = t;
        }
      }

      /*           Update the trailing submatrix */

      if (kstep == 1) {

        /*              1-by-1 pivot block D(k): column k now holds */

        /*              W(k) = L(k)*D(k) */

        /*              where L(k) is the k-th column of L */

        if (k < *n) {

          /*                 Perform a rank-1 update of A(k+1:n,k+1:n) as */

          /*                 A := A - L(k)*D(k)*L(k)' = A - W(k)*(1/D(k))*W(k)' */

          d11 = 1.f / a[k + k * a_dim1];
          i__1 = *n - k;
          r__1 = -d11;
          ssyr_(uplo, &i__1, &r__1, &a[k + 1 + k * a_dim1], &c__1, &
              a[k + 1 + (k + 1) * a_dim1], lda, (ftnlen)1);

          /*                 Store L(k) in column K */

          i__1 = *n - k;
          sscal_(&i__1, &d11, &a[k + 1 + k * a_dim1], &c__1);
        }
      } else {

        /*              2-by-2 pivot block D(k) */

        if (k < *n - 1) {

          /*                 Perform a rank-2 update of A(k+2:n,k+2:n) as */

          /*                 A := A - ( (A(k) A(k+1))*D(k)**(-1) ) * (A(k) A(k+1))' */

          /*                 where L(k) and L(k+1) are the k-th and (k+1)-th */
          /*                 columns of L */

          d21 = a[k + 1 + k * a_dim1];
          d11 = a[k + 1 + (k + 1) * a_dim1] / d21;
          d22 = a[k + k * a_dim1] / d21;
          t = 1.f / (d11 * d22 - 1.f);
          d21 = t / d21;

          i__1 = *n;
          for (j = k + 2; j <= i__1; ++j) {

            wk = d21 * (d11 * a[j + k * a_dim1] - a[j + (k + 1) * 
                a_dim1]);
            wkp1 = d21 * (d22 * a[j + (k + 1) * a_dim1] - a[j + k 
                * a_dim1]);

            i__2 = *n;
            for (i__ = j; i__ <= i__2; ++i__) {
              a[i__ + j * a_dim1] = a[i__ + j * a_dim1] - a[i__ 
                + k * a_dim1] * wk - a[i__ + (k + 1) * 
                a_dim1] * wkp1;
              /* L50: */
            }

            a[j + k * a_dim1] = wk;
            a[j + (k + 1) * a_dim1] = wkp1;

            /* L60: */
          }
        }
      }
    }

    /*        Store details of the interchanges in IPIV */

    if (kstep == 1) {
      ipiv[k] = kp;
    } else {
      ipiv[k] = -kp;
      ipiv[k + 1] = -kp;
    }

    /*        Increase K and return to the start of the main loop */

    k += kstep;
    goto L40;

  }

L70:

  return 0;

  /*     End of SSYTF2 */

} /* ssytf2_ */

/* ssytrf.f -- translated by f2c (version 20090411).
   You must link the resulting object file with libf2c:
   on Microsoft Windows system, link with libf2c.lib;
   on Linux or Unix systems, link with .../path/to/libf2c.a -lm
   or, if you install libf2c.a in a standard place, with -lf2c -lm
   -- in that order, at the end of the command line, as in
   cc *.o -lf2c -lm
   Source for libf2c is in /netlib/f2c/libf2c.zip, e.g.,

http://www.netlib.org/f2c/libf2c.zip
*/

/* Table of constant values */

static integer c__2 = 2;

/* Subroutine */ int ssytrf_(char *uplo, integer *n, real *a, integer *lda, 
    integer *ipiv, real *work, integer *lwork, integer *info, ftnlen 
    uplo_len)
{
  /* System generated locals */
  integer a_dim1, a_offset, i__1, i__2;

  /* Local variables */
  static integer j, k, kb, nb, iws;
  extern logical lsame_(char *, char *, ftnlen, ftnlen);
  static integer nbmin, iinfo;
  static logical upper;
  extern /* Subroutine */ int ssytf2_(char *, integer *, real *, integer *, 
      integer *, integer *, ftnlen);
  extern integer ilaenv_(integer *, char *, char *, integer *, integer *, 
      integer *, integer *, ftnlen, ftnlen);
  extern /* Subroutine */ int slasyf_(char *, integer *, integer *, integer 
      *, real *, integer *, integer *, real *, integer *, integer *, 
      ftnlen);
  static integer ldwork, lwkopt;
  static logical lquery;

  (void)(uplo_len);

  /*  -- LAPACK routine (version 3.2) -- */
  /*  -- LAPACK is a software package provided by Univ. of Tennessee,    -- */
  /*  -- Univ. of California Berkeley, Univ. of Colorado Denver and NAG Ltd..-- */
  /*     November 2006 */

  /*     .. Scalar Arguments .. */
  /*     .. */
  /*     .. Array Arguments .. */
  /*     .. */

  /*  Purpose */
  /*  ======= */

  /*  SSYTRF computes the factorization of a real symmetric matrix A using */
  /*  the Bunch-Kaufman diagonal pivoting method.  The form of the */
  /*  factorization is */

  /*     A = U*D*U**T  or  A = L*D*L**T */

  /*  where U (or L) is a product of permutation and unit upper (lower) */
  /*  triangular matrices, and D is symmetric and block diagonal with */
  /*  1-by-1 and 2-by-2 diagonal blocks. */

  /*  This is the blocked version of the algorithm, calling Level 3 BLAS. */

  /*  Arguments */
  /*  ========= */

  /*  UPLO    (input) CHARACTER*1 */
  /*          = 'U':  Upper triangle of A is stored; */
  /*          = 'L':  Lower triangle of A is stored. */

  /*  N       (input) INTEGER */
  /*          The order of the matrix A.  N >= 0. */

  /*  A       (input/output) REAL array, dimension (LDA,N) */
  /*          On entry, the symmetric matrix A.  If UPLO = 'U', the leading */
  /*          N-by-N upper triangular part of A contains the upper */
  /*          triangular part of the matrix A, and the strictly lower */
  /*          triangular part of A is not referenced.  If UPLO = 'L', the */
  /*          leading N-by-N lower triangular part of A contains the lower */
  /*          triangular part of the matrix A, and the strictly upper */
  /*          triangular part of A is not referenced. */

  /*          On exit, the block diagonal matrix D and the multipliers used */
  /*          to obtain the factor U or L (see below for further details). */

  /*  LDA     (input) INTEGER */
  /*          The leading dimension of the array A.  LDA >= max(1,N). */

  /*  IPIV    (output) INTEGER array, dimension (N) */
  /*          Details of the interchanges and the block structure of D. */
  /*          If IPIV(k) > 0, then rows and columns k and IPIV(k) were */
  /*          interchanged and D(k,k) is a 1-by-1 diagonal block. */
  /*          If UPLO = 'U' and IPIV(k) = IPIV(k-1) < 0, then rows and */
  /*          columns k-1 and -IPIV(k) were interchanged and D(k-1:k,k-1:k) */
  /*          is a 2-by-2 diagonal block.  If UPLO = 'L' and IPIV(k) = */
  /*          IPIV(k+1) < 0, then rows and columns k+1 and -IPIV(k) were */
  /*          interchanged and D(k:k+1,k:k+1) is a 2-by-2 diagonal block. */

  /*  WORK    (workspace/output) REAL array, dimension (MAX(1,LWORK)) */
  /*          On exit, if INFO = 0, WORK(1) returns the optimal LWORK. */

  /*  LWORK   (input) INTEGER */
  /*          The length of WORK.  LWORK >=1.  For best performance */
  /*          LWORK >= N*NB, where NB is the block size returned by ILAENV. */

  /*          If LWORK = -1, then a workspace query is assumed; the routine */
  /*          only calculates the optimal size of the WORK array, returns */
  /*          this value as the first entry of the WORK array, and no error */
  /*          message related to LWORK is issued by XERBLA. */

  /*  INFO    (output) INTEGER */
  /*          = 0:  successful exit */
  /*          < 0:  if INFO = -i, the i-th argument had an illegal value */
  /*          > 0:  if INFO = i, D(i,i) is exactly zero.  The factorization */
  /*                has been completed, but the block diagonal matrix D is */
  /*                exactly singular, and division by zero will occur if it */
  /*                is used to solve a system of equations. */

  /*  Further Details */
  /*  =============== */

  /*  If UPLO = 'U', then A = U*D*U', where */
  /*     U = P(n)*U(n)* ... *P(k)U(k)* ..., */
  /*  i.e., U is a product of terms P(k)*U(k), where k decreases from n to */
  /*  1 in steps of 1 or 2, and D is a block diagonal matrix with 1-by-1 */
  /*  and 2-by-2 diagonal blocks D(k).  P(k) is a permutation matrix as */
  /*  defined by IPIV(k), and U(k) is a unit upper triangular matrix, such */
  /*  that if the diagonal block D(k) is of order s (s = 1 or 2), then */

  /*             (   I    v    0   )   k-s */
  /*     U(k) =  (   0    I    0   )   s */
  /*             (   0    0    I   )   n-k */
  /*                k-s   s   n-k */

  /*  If s = 1, D(k) overwrites A(k,k), and v overwrites A(1:k-1,k). */
  /*  If s = 2, the upper triangle of D(k) overwrites A(k-1,k-1), A(k-1,k), */
  /*  and A(k,k), and v overwrites A(1:k-2,k-1:k). */

  /*  If UPLO = 'L', then A = L*D*L', where */
  /*     L = P(1)*L(1)* ... *P(k)*L(k)* ..., */
  /*  i.e., L is a product of terms P(k)*L(k), where k increases from 1 to */
  /*  n in steps of 1 or 2, and D is a block diagonal matrix with 1-by-1 */
  /*  and 2-by-2 diagonal blocks D(k).  P(k) is a permutation matrix as */
  /*  defined by IPIV(k), and L(k) is a unit lower triangular matrix, such */
  /*  that if the diagonal block D(k) is of order s (s = 1 or 2), then */

  /*             (   I    0     0   )  k-1 */
  /*     L(k) =  (   0    I     0   )  s */
  /*             (   0    v     I   )  n-k-s+1 */
  /*                k-1   s  n-k-s+1 */

  /*  If s = 1, D(k) overwrites A(k,k), and v overwrites A(k+1:n,k). */
  /*  If s = 2, the lower triangle of D(k) overwrites A(k,k), A(k+1,k), */
  /*  and A(k+1,k+1), and v overwrites A(k+2:n,k:k+1). */

  /*  ===================================================================== */

  /*     .. Local Scalars .. */
  /*     .. */
  /*     .. External Functions .. */
  /*     .. */
  /*     .. External Subroutines .. */
  /*     .. */
  /*     .. Intrinsic Functions .. */
  /*     .. */
  /*     .. Executable Statements .. */

  /*     Test the input parameters. */

  /* Parameter adjustments */
  a_dim1 = *lda;
  a_offset = 1 + a_dim1;
  a -= a_offset;
  --ipiv;
  --work;

  /* Function Body */
  *info = 0;
  upper = lsame_(uplo, "U", (ftnlen)1, (ftnlen)1);
  lquery = *lwork == -1;
  if (! upper && ! lsame_(uplo, "L", (ftnlen)1, (ftnlen)1)) {
    *info = -1;
  } else if (*n < 0) {
    *info = -2;
  } else if (*lda < max(1,*n)) {
    *info = -4;
  } else if (*lwork < 1 && ! lquery) {
    *info = -7;
  }

  if (*info == 0) {

    /*        Determine the block size */

    nb = ilaenv_(&c__1, "SSYTRF", uplo, n, &c_n1, &c_n1, &c_n1, (ftnlen)6,
        (ftnlen)1);
    lwkopt = *n * nb;
    work[1] = (real) lwkopt;
  }

  if (*info != 0) {
    i__1 = -(*info);
    return 0;
  } else if (lquery) {
    return 0;
  }

  nbmin = 2;
  ldwork = *n;
  if (nb > 1 && nb < *n) {
    iws = ldwork * nb;
    if (*lwork < iws) {
      /* Computing MAX */
      i__1 = *lwork / ldwork;
      nb = max(i__1,1);
      /* Computing MAX */
      i__1 = 2, i__2 = ilaenv_(&c__2, "SSYTRF", uplo, n, &c_n1, &c_n1, &
          c_n1, (ftnlen)6, (ftnlen)1);
      nbmin = max(i__1,i__2);
    }
  } else {
    iws = 1;
  }
  if (nb < nbmin) {
    nb = *n;
  }

  if (upper) {

    /*        Factorize A as U*D*U' using the upper triangle of A */

    /*        K is the main loop index, decreasing from N to 1 in steps of */
    /*        KB, where KB is the number of columns factorized by SLASYF; */
    /*        KB is either NB or NB-1, or K for the last block */

    k = *n;
L10:

    /*        If K < 1, exit from loop */

    if (k < 1) {
      goto L40;
    }

    if (k > nb) {

      /*           Factorize columns k-kb+1:k of A and use blocked code to */
      /*           update columns 1:k-kb */

      slasyf_(uplo, &k, &nb, &kb, &a[a_offset], lda, &ipiv[1], &work[1],
          &ldwork, &iinfo, (ftnlen)1);
    } else {

      /*           Use unblocked code to factorize columns 1:k of A */

      ssytf2_(uplo, &k, &a[a_offset], lda, &ipiv[1], &iinfo, (ftnlen)1);
      kb = k;
    }

    /*        Set INFO on the first occurrence of a zero pivot */

    if (*info == 0 && iinfo > 0) {
      *info = iinfo;
    }

    /*        Decrease K and return to the start of the main loop */

    k -= kb;
    goto L10;

  } else {

    /*        Factorize A as L*D*L' using the lower triangle of A */

    /*        K is the main loop index, increasing from 1 to N in steps of */
    /*        KB, where KB is the number of columns factorized by SLASYF; */
    /*        KB is either NB or NB-1, or N-K+1 for the last block */

    k = 1;
L20:

    /*        If K > N, exit from loop */

    if (k > *n) {
      goto L40;
    }

    if (k <= *n - nb) {

      /*           Factorize columns k:k+kb-1 of A and use blocked code to */
      /*           update columns k+kb:n */

      i__1 = *n - k + 1;
      slasyf_(uplo, &i__1, &nb, &kb, &a[k + k * a_dim1], lda, &ipiv[k], 
          &work[1], &ldwork, &iinfo, (ftnlen)1);
    } else {

      /*           Use unblocked code to factorize columns k:n of A */

      i__1 = *n - k + 1;
      ssytf2_(uplo, &i__1, &a[k + k * a_dim1], lda, &ipiv[k], &iinfo, (
            ftnlen)1);
      kb = *n - k + 1;
    }

    /*        Set INFO on the first occurrence of a zero pivot */

    if (*info == 0 && iinfo > 0) {
      *info = iinfo + k - 1;
    }

    /*        Adjust IPIV */

    i__1 = k + kb - 1;
    for (j = k; j <= i__1; ++j) {
      if (ipiv[j] > 0) {
        ipiv[j] = ipiv[j] + k - 1;
      } else {
        ipiv[j] = ipiv[j] - k + 1;
      }
      /* L30: */
    }

    /*        Increase K and return to the start of the main loop */

    k += kb;
    goto L20;

  }

L40:
  work[1] = (real) lwkopt;
  return 0;

  /*     End of SSYTRF */

} /* ssytrf_ */

/* ssytrs.f -- translated by f2c (version 20090411).
   You must link the resulting object file with libf2c:
   on Microsoft Windows system, link with libf2c.lib;
   on Linux or Unix systems, link with .../path/to/libf2c.a -lm
   or, if you install libf2c.a in a standard place, with -lf2c -lm
   -- in that order, at the end of the command line, as in
   cc *.o -lf2c -lm
   Source for libf2c is in /netlib/f2c/libf2c.zip, e.g.,

http://www.netlib.org/f2c/libf2c.zip
*/

/* Table of constant values */

static real c_b7 = -1.f;
static real c_b19 = 1.f;

/* Subroutine */ int ssytrs_(char *uplo, integer *n, integer *nrhs, real *a, 
    integer *lda, integer *ipiv, real *b, integer *ldb, integer *info, 
    ftnlen uplo_len)
{
  /* System generated locals */
  integer a_dim1, a_offset, b_dim1, b_offset, i__1;
  real r__1;

  /* Local variables */
  static integer j, k;
  static real ak, bk;
  static integer kp;
  static real akm1, bkm1;
  extern /* Subroutine */ int sger_(integer *, integer *, real *, real *, 
      integer *, real *, integer *, real *, integer *);
  static real akm1k;
  extern logical lsame_(char *, char *, ftnlen, ftnlen);
  static real denom;
  extern /* Subroutine */ int sscal_(integer *, real *, real *, integer *), 
         sgemv_(char *, integer *, integer *, real *, real *, integer *, 
             real *, integer *, real *, real *, integer *, ftnlen);
  static logical upper;
  extern /* Subroutine */ int sswap_(integer *, real *, integer *, real *, 
      integer *);


  (void)(uplo_len);
  /*  -- LAPACK routine (version 3.2) -- */
  /*  -- LAPACK is a software package provided by Univ. of Tennessee,    -- */
  /*  -- Univ. of California Berkeley, Univ. of Colorado Denver and NAG Ltd..-- */
  /*     November 2006 */

  /*     .. Scalar Arguments .. */
  /*     .. */
  /*     .. Array Arguments .. */
  /*     .. */

  /*  Purpose */
  /*  ======= */

  /*  SSYTRS solves a system of linear equations A*X = B with a real */
  /*  symmetric matrix A using the factorization A = U*D*U**T or */
  /*  A = L*D*L**T computed by SSYTRF. */

  /*  Arguments */
  /*  ========= */

  /*  UPLO    (input) CHARACTER*1 */
  /*          Specifies whether the details of the factorization are stored */
  /*          as an upper or lower triangular matrix. */
  /*          = 'U':  Upper triangular, form is A = U*D*U**T; */
  /*          = 'L':  Lower triangular, form is A = L*D*L**T. */

  /*  N       (input) INTEGER */
  /*          The order of the matrix A.  N >= 0. */

  /*  NRHS    (input) INTEGER */
  /*          The number of right hand sides, i.e., the number of columns */
  /*          of the matrix B.  NRHS >= 0. */

  /*  A       (input) REAL array, dimension (LDA,N) */
  /*          The block diagonal matrix D and the multipliers used to */
  /*          obtain the factor U or L as computed by SSYTRF. */

  /*  LDA     (input) INTEGER */
  /*          The leading dimension of the array A.  LDA >= max(1,N). */

  /*  IPIV    (input) INTEGER array, dimension (N) */
  /*          Details of the interchanges and the block structure of D */
  /*          as determined by SSYTRF. */

  /*  B       (input/output) REAL array, dimension (LDB,NRHS) */
  /*          On entry, the right hand side matrix B. */
  /*          On exit, the solution matrix X. */

  /*  LDB     (input) INTEGER */
  /*          The leading dimension of the array B.  LDB >= max(1,N). */

  /*  INFO    (output) INTEGER */
  /*          = 0:  successful exit */
  /*          < 0:  if INFO = -i, the i-th argument had an illegal value */

  /*  ===================================================================== */

  /*     .. Parameters .. */
  /*     .. */
  /*     .. Local Scalars .. */
  /*     .. */
  /*     .. External Functions .. */
  /*     .. */
  /*     .. External Subroutines .. */
  /*     .. */
  /*     .. Intrinsic Functions .. */
  /*     .. */
  /*     .. Executable Statements .. */

  /* Parameter adjustments */
  a_dim1 = *lda;
  a_offset = 1 + a_dim1;
  a -= a_offset;
  --ipiv;
  b_dim1 = *ldb;
  b_offset = 1 + b_dim1;
  b -= b_offset;

  /* Function Body */
  *info = 0;
  upper = lsame_(uplo, "U", (ftnlen)1, (ftnlen)1);
  if (! upper && ! lsame_(uplo, "L", (ftnlen)1, (ftnlen)1)) {
    *info = -1;
  } else if (*n < 0) {
    *info = -2;
  } else if (*nrhs < 0) {
    *info = -3;
  } else if (*lda < max(1,*n)) {
    *info = -5;
  } else if (*ldb < max(1,*n)) {
    *info = -8;
  }
  if (*info != 0) {
    i__1 = -(*info);
    return 0;
  }

  /*     Quick return if possible */

  if (*n == 0 || *nrhs == 0) {
    return 0;
  }

  if (upper) {

    /*        Solve A*X = B, where A = U*D*U'. */

    /*        First solve U*D*X = B, overwriting B with X. */

    /*        K is the main loop index, decreasing from N to 1 in steps of */
    /*        1 or 2, depending on the size of the diagonal blocks. */

    k = *n;
L10:

    /*        If K < 1, exit from loop. */

    if (k < 1) {
      goto L30;
    }

    if (ipiv[k] > 0) {

      /*           1 x 1 diagonal block */

      /*           Interchange rows K and IPIV(K). */

      kp = ipiv[k];
      if (kp != k) {
        sswap_(nrhs, &b[k + b_dim1], ldb, &b[kp + b_dim1], ldb);
      }

      /*           Multiply by inv(U(K)), where U(K) is the transformation */
      /*           stored in column K of A. */

      i__1 = k - 1;
      sger_(&i__1, nrhs, &c_b7, &a[k * a_dim1 + 1], &c__1, &b[k + 
          b_dim1], ldb, &b[b_dim1 + 1], ldb);

      /*           Multiply by the inverse of the diagonal block. */

      r__1 = 1.f / a[k + k * a_dim1];
      sscal_(nrhs, &r__1, &b[k + b_dim1], ldb);
      --k;
    } else {

      /*           2 x 2 diagonal block */

      /*           Interchange rows K-1 and -IPIV(K). */

      kp = -ipiv[k];
      if (kp != k - 1) {
        sswap_(nrhs, &b[k - 1 + b_dim1], ldb, &b[kp + b_dim1], ldb);
      }

      /*           Multiply by inv(U(K)), where U(K) is the transformation */
      /*           stored in columns K-1 and K of A. */

      i__1 = k - 2;
      sger_(&i__1, nrhs, &c_b7, &a[k * a_dim1 + 1], &c__1, &b[k + 
          b_dim1], ldb, &b[b_dim1 + 1], ldb);
      i__1 = k - 2;
      sger_(&i__1, nrhs, &c_b7, &a[(k - 1) * a_dim1 + 1], &c__1, &b[k - 
          1 + b_dim1], ldb, &b[b_dim1 + 1], ldb);

      /*           Multiply by the inverse of the diagonal block. */

      akm1k = a[k - 1 + k * a_dim1];
      akm1 = a[k - 1 + (k - 1) * a_dim1] / akm1k;
      ak = a[k + k * a_dim1] / akm1k;
      denom = akm1 * ak - 1.f;
      i__1 = *nrhs;
      for (j = 1; j <= i__1; ++j) {
        bkm1 = b[k - 1 + j * b_dim1] / akm1k;
        bk = b[k + j * b_dim1] / akm1k;
        b[k - 1 + j * b_dim1] = (ak * bkm1 - bk) / denom;
        b[k + j * b_dim1] = (akm1 * bk - bkm1) / denom;
        /* L20: */
      }
      k += -2;
    }

    goto L10;
L30:

    /*        Next solve U'*X = B, overwriting B with X. */

    /*        K is the main loop index, increasing from 1 to N in steps of */
    /*        1 or 2, depending on the size of the diagonal blocks. */

    k = 1;
L40:

    /*        If K > N, exit from loop. */

    if (k > *n) {
      goto L50;
    }

    if (ipiv[k] > 0) {

      /*           1 x 1 diagonal block */

      /*           Multiply by inv(U'(K)), where U(K) is the transformation */
      /*           stored in column K of A. */

      i__1 = k - 1;
      sgemv_("Transpose", &i__1, nrhs, &c_b7, &b[b_offset], ldb, &a[k * 
          a_dim1 + 1], &c__1, &c_b19, &b[k + b_dim1], ldb, (ftnlen)
          9);

      /*           Interchange rows K and IPIV(K). */

      kp = ipiv[k];
      if (kp != k) {
        sswap_(nrhs, &b[k + b_dim1], ldb, &b[kp + b_dim1], ldb);
      }
      ++k;
    } else {

      /*           2 x 2 diagonal block */

      /*           Multiply by inv(U'(K+1)), where U(K+1) is the transformation */
      /*           stored in columns K and K+1 of A. */

      i__1 = k - 1;
      sgemv_("Transpose", &i__1, nrhs, &c_b7, &b[b_offset], ldb, &a[k * 
          a_dim1 + 1], &c__1, &c_b19, &b[k + b_dim1], ldb, (ftnlen)
          9);
      i__1 = k - 1;
      sgemv_("Transpose", &i__1, nrhs, &c_b7, &b[b_offset], ldb, &a[(k 
            + 1) * a_dim1 + 1], &c__1, &c_b19, &b[k + 1 + b_dim1], 
          ldb, (ftnlen)9);

      /*           Interchange rows K and -IPIV(K). */

      kp = -ipiv[k];
      if (kp != k) {
        sswap_(nrhs, &b[k + b_dim1], ldb, &b[kp + b_dim1], ldb);
      }
      k += 2;
    }

    goto L40;
L50:

    ;
  } else {

    /*        Solve A*X = B, where A = L*D*L'. */

    /*        First solve L*D*X = B, overwriting B with X. */

    /*        K is the main loop index, increasing from 1 to N in steps of */
    /*        1 or 2, depending on the size of the diagonal blocks. */

    k = 1;
L60:

    /*        If K > N, exit from loop. */

    if (k > *n) {
      goto L80;
    }

    if (ipiv[k] > 0) {

      /*           1 x 1 diagonal block */

      /*           Interchange rows K and IPIV(K). */

      kp = ipiv[k];
      if (kp != k) {
        sswap_(nrhs, &b[k + b_dim1], ldb, &b[kp + b_dim1], ldb);
      }

      /*           Multiply by inv(L(K)), where L(K) is the transformation */
      /*           stored in column K of A. */

      if (k < *n) {
        i__1 = *n - k;
        sger_(&i__1, nrhs, &c_b7, &a[k + 1 + k * a_dim1], &c__1, &b[k 
            + b_dim1], ldb, &b[k + 1 + b_dim1], ldb);
      }

      /*           Multiply by the inverse of the diagonal block. */

      r__1 = 1.f / a[k + k * a_dim1];
      sscal_(nrhs, &r__1, &b[k + b_dim1], ldb);
      ++k;
    } else {

      /*           2 x 2 diagonal block */

      /*           Interchange rows K+1 and -IPIV(K). */

      kp = -ipiv[k];
      if (kp != k + 1) {
        sswap_(nrhs, &b[k + 1 + b_dim1], ldb, &b[kp + b_dim1], ldb);
      }

      /*           Multiply by inv(L(K)), where L(K) is the transformation */
      /*           stored in columns K and K+1 of A. */

      if (k < *n - 1) {
        i__1 = *n - k - 1;
        sger_(&i__1, nrhs, &c_b7, &a[k + 2 + k * a_dim1], &c__1, &b[k 
            + b_dim1], ldb, &b[k + 2 + b_dim1], ldb);
        i__1 = *n - k - 1;
        sger_(&i__1, nrhs, &c_b7, &a[k + 2 + (k + 1) * a_dim1], &c__1,
            &b[k + 1 + b_dim1], ldb, &b[k + 2 + b_dim1], ldb);
      }

      /*           Multiply by the inverse of the diagonal block. */

      akm1k = a[k + 1 + k * a_dim1];
      akm1 = a[k + k * a_dim1] / akm1k;
      ak = a[k + 1 + (k + 1) * a_dim1] / akm1k;
      denom = akm1 * ak - 1.f;
      i__1 = *nrhs;
      for (j = 1; j <= i__1; ++j) {
        bkm1 = b[k + j * b_dim1] / akm1k;
        bk = b[k + 1 + j * b_dim1] / akm1k;
        b[k + j * b_dim1] = (ak * bkm1 - bk) / denom;
        b[k + 1 + j * b_dim1] = (akm1 * bk - bkm1) / denom;
        /* L70: */
      }
      k += 2;
    }

    goto L60;
L80:

    /*        Next solve L'*X = B, overwriting B with X. */

    /*        K is the main loop index, decreasing from N to 1 in steps of */
    /*        1 or 2, depending on the size of the diagonal blocks. */

    k = *n;
L90:

    /*        If K < 1, exit from loop. */

    if (k < 1) {
      goto L100;
    }

    if (ipiv[k] > 0) {

      /*           1 x 1 diagonal block */

      /*           Multiply by inv(L'(K)), where L(K) is the transformation */
      /*           stored in column K of A. */

      if (k < *n) {
        i__1 = *n - k;
        sgemv_("Transpose", &i__1, nrhs, &c_b7, &b[k + 1 + b_dim1], 
            ldb, &a[k + 1 + k * a_dim1], &c__1, &c_b19, &b[k + 
            b_dim1], ldb, (ftnlen)9);
      }

      /*           Interchange rows K and IPIV(K). */

      kp = ipiv[k];
      if (kp != k) {
        sswap_(nrhs, &b[k + b_dim1], ldb, &b[kp + b_dim1], ldb);
      }
      --k;
    } else {

      /*           2 x 2 diagonal block */

      /*           Multiply by inv(L'(K-1)), where L(K-1) is the transformation */
      /*           stored in columns K-1 and K of A. */

      if (k < *n) {
        i__1 = *n - k;
        sgemv_("Transpose", &i__1, nrhs, &c_b7, &b[k + 1 + b_dim1], 
            ldb, &a[k + 1 + k * a_dim1], &c__1, &c_b19, &b[k + 
            b_dim1], ldb, (ftnlen)9);
        i__1 = *n - k;
        sgemv_("Transpose", &i__1, nrhs, &c_b7, &b[k + 1 + b_dim1], 
            ldb, &a[k + 1 + (k - 1) * a_dim1], &c__1, &c_b19, &b[
            k - 1 + b_dim1], ldb, (ftnlen)9);
      }

      /*           Interchange rows K and -IPIV(K). */

      kp = -ipiv[k];
      if (kp != k) {
        sswap_(nrhs, &b[k + b_dim1], ldb, &b[kp + b_dim1], ldb);
      }
      k += -2;
    }

    goto L90;
L100:
    ;
  }

  return 0;

  /*     End of SSYTRS */

} /* ssytrs_ */

