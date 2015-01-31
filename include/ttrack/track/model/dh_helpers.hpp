#ifndef __DH_HELPERS__
#define __DH_HELPERS__

#include <cinder/Matrix.h>
#include <cinder/gl/gl.h>

// OpenGL matrix offsets
#define _00 0
#define _10 1
#define _20 2
#define _30 3
#define _01 4
#define _11 5
#define _21 6
#define _31 7
#define _02 8
#define _12 9
#define _22 10
#define _32 11
#define _03 12
#define _13 13
#define _23 14
#define _33 15

inline ci::Matrix44f glhMultMatrixRight(const ci::Matrix44f &A, ci::Matrix44f &B)
{

  GLfloat M[16];
  M[_00] = B[_00] * A[_00] + B[_01] * A[_10] + B[_02] * A[_20] + B[_03] * A[_30];
  M[_10] = B[_10] * A[_00] + B[_11] * A[_10] + B[_12] * A[_20] + B[_13] * A[_30];
  M[_20] = B[_20] * A[_00] + B[_21] * A[_10] + B[_22] * A[_20] + B[_23] * A[_30];
  M[_30] = B[_30] * A[_00] + B[_31] * A[_10] + B[_32] * A[_20] + B[_33] * A[_30];
  M[_01] = B[_00] * A[_01] + B[_01] * A[_11] + B[_02] * A[_21] + B[_03] * A[_31];
  M[_11] = B[_10] * A[_01] + B[_11] * A[_11] + B[_12] * A[_21] + B[_13] * A[_31];
  M[_21] = B[_20] * A[_01] + B[_21] * A[_11] + B[_22] * A[_21] + B[_23] * A[_31];
  M[_31] = B[_30] * A[_01] + B[_31] * A[_11] + B[_32] * A[_21] + B[_33] * A[_31];
  M[_02] = B[_00] * A[_02] + B[_01] * A[_12] + B[_02] * A[_22] + B[_03] * A[_32];
  M[_12] = B[_10] * A[_02] + B[_11] * A[_12] + B[_12] * A[_22] + B[_13] * A[_32];
  M[_22] = B[_20] * A[_02] + B[_21] * A[_12] + B[_22] * A[_22] + B[_23] * A[_32];
  M[_32] = B[_30] * A[_02] + B[_31] * A[_12] + B[_32] * A[_22] + B[_33] * A[_32];
  M[_03] = B[_00] * A[_03] + B[_01] * A[_13] + B[_02] * A[_23] + B[_03] * A[_33];
  M[_13] = B[_10] * A[_03] + B[_11] * A[_13] + B[_12] * A[_23] + B[_13] * A[_33];
  M[_23] = B[_20] * A[_03] + B[_21] * A[_13] + B[_22] * A[_23] + B[_23] * A[_33];
  M[_33] = B[_30] * A[_03] + B[_31] * A[_13] + B[_32] * A[_23] + B[_33] * A[_33];
  for (unsigned int i = 0; i < 16; ++i)
    B[i] = M[i];

  return B;
}

inline void glhDenavitHartenberg(GLfloat a, GLfloat alpha, GLfloat d, GLfloat theta, GLfloat* A) {

  GLfloat sa = sin(alpha);
  GLfloat ca = cos(alpha);
  GLfloat st = sin(theta);
  GLfloat ct = cos(theta);

  A[_00] = ct;
  A[_10] = ca * st;
  A[_20] = sa * st;
  A[_30] = 0.0;
  A[_01] = -st;
  A[_11] = ca * ct;
  A[_21] = sa * ct;
  A[_31] = 0.0;
  A[_02] = 0.0;
  A[_12] = -sa;
  A[_22] = ca;
  A[_32] = 0.0;
  A[_03] = a * 1000;
  A[_13] = -sa * d * 1000;
  A[_23] = ca * d * 1000;
  A[_33] = 1.0;

}


inline void glhDenavitHartenbergDerivative(GLfloat a, GLfloat alpha, GLfloat d, GLfloat theta, GLfloat* A) {

  GLfloat sa = sin(alpha);
  GLfloat ca = cos(alpha);
  GLfloat st = sin(theta);
  GLfloat ct = cos(theta);

  A[_00] = -st;
  A[_10] = ca * ct;
  A[_20] = sa * ct;
  A[_30] = 0.0;
  A[_01] = -ct;
  A[_11] = -ca * st;
  A[_21] = -sa * st;
  A[_31] = 0.0;
  A[_02] = 0.0;
  A[_12] = -sa;
  A[_22] = ca;
  A[_32] = 0.0;
  A[_03] = 0;
  A[_13] = 0;
  A[_23] = 0;
  A[_33] = 1.0;

}

#endif