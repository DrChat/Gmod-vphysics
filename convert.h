#ifndef CONVERT_H
#define CONVERT_H

#define HL2BULL_FACTOR METERS_PER_INCH
#define BULL2HL(x) (float)(x * (1.0f/HL2BULL_FACTOR))
#define HL2BULL(x) (float)(x * HL2BULL_FACTOR)

btCompoundShape* ConvertMeshToBull(CPhysCollide* ivp);

void ConvertPosToBull(const Vector& pos, btVector3& bull);
void ConvertPosToHL(const btVector3& pos, Vector& hl);
void ConvertRotationToBull(const QAngle& angles, btMatrix3x3& bull);
void ConvertRotationToBull(const QAngle& angles, btQuaternion& bull);
void ConvertRotationToHL(const btMatrix3x3& matrix, QAngle& hl);
void ConvertRotationToHL(const btQuaternion& quat, QAngle& hl);

#endif
