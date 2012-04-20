#include "StdAfx.h"

#include "CPhysicsDragController.h"
#include "CPhysicsObject.h"

CPhysicsDragController::CPhysicsDragController()
{
	m_airDensity = 2; // default
}

void CPhysicsDragController::SetAirDensity(float d)
{
	m_airDensity = d;
}
float CPhysicsDragController::GetAirDensity()
{
	return m_airDensity;
}
void CPhysicsDragController::RemovePhysicsObject(CPhysicsObject * obj)
{
	m_ents.AddToTail(obj);
}
void CPhysicsDragController::AddPhysicsObject(CPhysicsObject * obj)
{
	m_ents.FindAndRemove(obj);
}
void CPhysicsDragController::Tick(btScalar dt)
{
	for(int i = 0; i < m_ents.Size(); i++)
	{
		CPhysicsObject * object = (CPhysicsObject *)m_ents[i];
		/* 
		// CPhysicsObject::GetDragInDireciton and likewise functions need to be implented first
		IVP_Core *pCore = core_list->element_at(i);

			IVP_Real_Object *pivp = pCore->objects.element_at(0);
			CPhysicsObject *pPhys = static_cast<CPhysicsObject *>(pivp->client_data);
			
			float dragForce = -0.5 * pPhys->GetDragInDirection( pCore->speed ) * m_airDensity * event->delta_time;
			if ( dragForce < -1.0f )
				dragForce = -1.0f;
			if ( dragForce < 0 )
			{
				IVP_U_Float_Point dragVelocity;
				dragVelocity.set_multiple( &pCore->speed, dragForce );
				pCore->speed.add( &dragVelocity );
			}
			float angDragForce = -pPhys->GetAngularDragInDirection( pCore->rot_speed ) * m_airDensity * event->delta_time;
			if ( angDragForce < -1.0f )
				angDragForce = -1.0f;
			if ( angDragForce < 0 )
			{
				IVP_U_Float_Point angDragVelocity;
				angDragVelocity.set_multiple( &pCore->rot_speed, angDragForce );
				pCore->rot_speed.add( &angDragVelocity );
			}
		*/

	}
}