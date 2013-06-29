#ifndef CONVERT_H
#define CONVERT_H

#define HL2BULL_FACTOR METERS_PER_INCH
#define HL2BULL_INSQR_PER_METERSQR (1.f / (HL2BULL_FACTOR*HL2BULL_FACTOR))

#define BULL2HL(x) (float)((x) * (1.0f/HL2BULL_FACTOR))
#define HL2BULL(x) (float)((x) * HL2BULL_FACTOR)

//btCompoundShape *ConvertMeshToBull(CPhysCollide *ivp);

void ConvertIVPPosToBull(const float *pos, btVector3 &bull);
void ConvertPosToBull(const Vector &pos, btVector3 &bull);
void ConvertPosToHL(const btVector3 &pos, Vector &hl);
void ConvertAABBToBull(const Vector &hlMins, const Vector &hlMaxs, btVector3 &bullMins, btVector3 &bullMaxs);
void ConvertAABBToHL(const btVector3 &bullMins, const btVector3 &bullMaxs, Vector &hlMins, Vector &hlMaxs);
void ConvertDirectionToBull(const Vector &dir, btVector3 &bull);
void ConvertDirectionToHL(const btVector3 &dir, Vector &hl);
void ConvertForceImpulseToBull(const Vector &pos, btVector3 &bull);
void ConvertForceImpulseToHL(const btVector3 &pos, Vector &hl);
void ConvertForceImpulseToBull(const float &hl, btScalar &bull);
void ConvertForceImpulseToHL(const btScalar &bull, float &hl);
void ConvertRotationToBull(const QAngle &angles, btMatrix3x3 &bull);
void ConvertRotationToBull(const QAngle &angles, btQuaternion &bull);
void ConvertRotationToHL(const btMatrix3x3 &matrix, QAngle &hl);
void ConvertRotationToHL(const btQuaternion &quat, QAngle &hl);
void ConvertAngularImpulseToBull(const AngularImpulse &angularimp, btVector3 &bull);
void ConvertAngularImpulseToHL(const btVector3 &angularimp, AngularImpulse &hl);
void ConvertMatrixToHL(const btTransform &transform, matrix3x4_t &hl);
void ConvertMatrixToBull(const matrix3x4_t &hl, btTransform &transform);

float ConvertDistanceToBull(float distance);
float ConvertDistanceToHL(float distance);
float ConvertEnergyToHL(float energy);

#endif
