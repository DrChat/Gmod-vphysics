#include <tier1/tier1.h>
#include <vphysics_interface.h>

#define TEST(name) printf(#name ": %s\n", name() ? "Success!" : "Failure!")

CSysModule* g_vphysics = NULL;
IPhysics* g_Physics = NULL;
IPhysicsSurfaceProps* g_PhysicsSurfaceProps = NULL;
IPhysicsCollision* g_PhysicsCollision = NULL;

IPhysicsEnvironment* g_Env = NULL;
CPhysCollide* g_Collide = NULL;
IPhysicsObject* g_Object1 = NULL;

bool Init() {
	g_vphysics = Sys_LoadModule("vphysics");
	CreateInterfaceFn factory = Sys_GetFactory(g_vphysics);
	if (!factory) return false;

	g_Physics = (IPhysics*)factory(VPHYSICS_INTERFACE_VERSION, NULL);
	g_PhysicsSurfaceProps = (IPhysicsSurfaceProps*)factory(VPHYSICS_SURFACEPROPS_INTERFACE_VERSION, NULL);
	g_PhysicsCollision = (IPhysicsCollision*)factory(VPHYSICS_COLLISION_INTERFACE_VERSION, NULL);

	g_Physics->Init();
	//g_Physics->Connect();

	FILE* file = fopen("surfaceproperties.txt", "rb");
	fseek(file, 0, SEEK_END);
	int size = ftell(file);
	fseek(file, 0, SEEK_SET);
	char* buffer = (char*)malloc(size+1);
	fread(buffer, sizeof(char), size, file);
	buffer[size] = 0;
	fclose(file);

	g_PhysicsSurfaceProps->ParseSurfaceData("surfaceproperties.txt", buffer);
	free(buffer);

	g_Env = g_Physics->CreateEnvironment();

	g_Collide = g_PhysicsCollision->BBoxToCollide(Vector(-16, -16, 0), Vector(16, 16, 72));

	int surfaceindex = g_PhysicsSurfaceProps->GetSurfaceIndex("default");
	objectparams_t params = {NULL, 1.0f, 1.0f, 0.1f, 0.1f, 0.05f, "default", NULL, 0.0f, 1.0f, true};
	g_Object1 = g_Env->CreatePolyObject(g_Collide, surfaceindex, Vector(0,0,0), QAngle(0,0,0), &params);

	return true;
}

void Shutdown() {
	g_Env->DestroyObject(g_Object1);
	g_PhysicsCollision->DestroyCollide(g_Collide);
	g_Physics->DestroyEnvironment(g_Env);

	//g_Physics->Disconnect();
	g_Physics->Shutdown();

	Sys_UnloadModule(g_vphysics);
	g_vphysics = NULL;
}

bool BasicPosition() {
	Vector pos1, pos2;
	pos1.Random(-4096, 4096);
	g_Object1->SetPosition(pos1, QAngle(0,0,0), true);
	g_Object1->GetPosition(&pos2, NULL);
	return pos1.DistTo(pos2) < 1e-5;
}

bool BasicAngle() {
	QAngle rot;
	matrix3x4_t matrix1, matrix2;
	rot.Random(-180, 180);
	AngleMatrix(rot, Vector(0,0,0), matrix1);
	g_Object1->SetPosition(Vector(0,0,0), rot, true);
	g_Object1->GetPosition(NULL, &rot);
	AngleMatrix(rot, Vector(0,0,0), matrix2);
	return MatricesAreEqual(matrix1, matrix2);
}

bool BasicMatrix() {
	Vector pos;
	QAngle rot;
	matrix3x4_t matrix1, matrix2;
	pos.Random(-4096, 4096);
	rot.Random(-180, 180);
	AngleMatrix(rot, pos, matrix1);
	g_Object1->SetPositionMatrix(matrix1, true);
	g_Object1->GetPositionMatrix(&matrix2);
	return MatricesAreEqual(matrix1, matrix2);
}

bool GetMatrix() {
	Vector pos;
	QAngle rot;
	matrix3x4_t matrix1, matrix2;
	pos.Random(-4096, 4096);
	rot.Random(-180, 180);
	AngleMatrix(rot, pos, matrix1);
	g_Object1->SetPosition(pos, rot, true);
	g_Object1->GetPositionMatrix(&matrix2);
	return MatricesAreEqual(matrix1, matrix2);
}

bool SetMatrix() {
	Vector pos;
	QAngle rot;
	matrix3x4_t matrix1, matrix2;
	pos.Random(-4096, 4096);
	rot.Random(-180, 180);
	AngleMatrix(rot, pos, matrix1);
	g_Object1->SetPositionMatrix(matrix1, true);
	g_Object1->GetPosition(&pos, &rot);
	AngleMatrix(rot, pos, matrix2);
	return MatricesAreEqual(matrix1, matrix2);
}

int main(int argc, char* argv[]) {
	Init();

	TEST(BasicPosition);
	TEST(BasicAngle);
	TEST(BasicMatrix);
	TEST(GetMatrix);
	TEST(SetMatrix);

	Shutdown();
}
