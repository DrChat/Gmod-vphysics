#ifndef MATH_H
#define MATH_H

void BtMatrix_vimult(btMatrix3x3 * matrix, btVector3 * in, btVector3 * out);
float AngDragIntegral( float invInertia, float l, float w, float h );

#endif