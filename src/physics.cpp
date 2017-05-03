#include <imgui\imgui.h>
#include <imgui\imgui_impl_glfw_gl3.h>
#include <glm\gtc\type_ptr.hpp>
#include <glm\gtc\matrix_transform.hpp>

#include <time.h>
#include <stdio.h>
bool show_test_window = false;
using namespace glm;
float reset;
vec3 gravity = vec3(0, -9.8f, 0);
vec3 verticalForce;
vec3 linearMomentum;
vec3 angularMomentum;
vec3 torque;
vec3 velocity;
vec3 position;
mat4 rotatedMatrix;
mat4 translatedMatrix;
mat3 Ibody;
quat quaternion;
mat3 inertiaInvers;
vec3 angularVelocity;
float mass = 1;
float xpos;
float ypos;
float zpos;
float tolerance;
vec4 cubeUpdateVertex[7];
float restitutionCoefficient = 0.5f;

vec4 CubesPositionBuffer[] = {
	vec4(-1.f,  -1.f, -1.f,1.f),
	vec4(1.f,  -1.f, -1.f,1.f),
	vec4(1.f,  -1.f,  1.f,1.f),
	vec4(-1.f,  -1.f,  1.f,1.f),
	vec4( - 1.f, 1.f, -1.f,1.f),
	vec4(1.f, 1.f, -1.f,1.f),
	vec4(1.f, 1.f,  1.f,1.f),
	vec4(-1.f, 1.f,  1.f,1.f)
};
namespace Cube {

	extern void setupCube();
	extern void cleanupCube();
	extern void updateCube(glm::mat4x4);
	extern void drawCube();
	
}

void GUI() {
	{	//FrameRate
		ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

		//TODO
	}

	// ImGui test window. Most of the sample code is in ImGui::ShowTestWindow()
	if(show_test_window) {
		ImGui::SetNextWindowPos(ImVec2(650, 20), ImGuiSetCond_FirstUseEver);
		ImGui::ShowTestWindow(&show_test_window);
	}
}
void collisionPlane(vec3 cubeVertex, mat3 inverseMatrix, vec3 normalPlane, vec3 velocity, vec3 angularVelocity,vec3 massCenter) {
	vec3 collisionVertex = velocity + cross(angularVelocity, (cubeVertex-massCenter));
	float vRel = dot(normalPlane, collisionVertex);
	float miniImpulse = (-(1 + restitutionCoefficient)*vRel) / (1 / mass) + dot(normalPlane,cross((inverseMatrix*(cross(cubeVertex, normalPlane))), cubeVertex));
	vec3 Impulse = miniImpulse*normalPlane;
	vec3 torqueImpulse = cross(cubeVertex, Impulse);
	linearMomentum = linearMomentum + Impulse;
	angularMomentum = angularMomentum + torqueImpulse;
}
void PhysicsInit() {
	//TODO
	quaternion = quat(0,0,0,0);
	tolerance = 0.1f;

	reset = 5;
	srand(time(NULL));
	glm::mat4 matriu = {//matriu Original
		1,0,0,0,
		0,1,0,0,
		0,0,1,0,
		0,0,0,1
	};
	rotatedMatrix = matriu;
	//posicions random
	xpos = ((float)rand() / RAND_MAX) * 5 - 4;
	ypos = ((float)rand() / RAND_MAX) * 9 + 2;
	zpos = ((float)rand() / RAND_MAX) * 5 - 4;
	//matriu translacio
	translatedMatrix = translate(matriu, vec3(xpos, ypos, zpos));
	//rotacions random
	float randAngle = radians((((float)rand() / RAND_MAX) * 359) + 30) ;
	int xrot = ((float)rand() / RAND_MAX) *10 -5;
	int yrot = ((float)rand() / RAND_MAX) *10 -5;
	int zrot = ((float)rand() / RAND_MAX) *10 -5;

	/*printf("%d", xrot);
	printf("%d", yrot);
	printf("%d\n", zrot);
	printf("%f\n", degrees(randAngle));*/
	//matriu rotacio i translacio
	//rotatedMatrix = rotate(matriu, randAngle, glm::vec3(1, 1, 1));
	//matriu = rotate(translatedMatrix, randAngle, glm::vec3(xrot, yrot, zrot));
	//matriu rotacio
	 
	//vertical force random
	verticalForce = vec3((((float)rand() / RAND_MAX) * 4) -2, (((float)rand() / RAND_MAX) * 8)+2, (((float)rand() / RAND_MAX) * 4) - 2);
	//moment lineal
	linearMomentum = vec3(0) + verticalForce;
	//vectors torque
	vec3 forcePoint = vec3(xrot,yrot,zrot);
	vec3 vecForce = forcePoint - vec3(0);
	//torque
	torque = cross(vecForce, verticalForce);
	
	//moment angular
	angularMomentum = vec3(0) + torque;
	
	//velocitat
	velocity = linearMomentum / mass;

	//posicio
	position = vec3(xpos,ypos,zpos);
	//updatecube
	Cube::updateCube(matriu);
	//matriu inertia body
	
}
void PhysicsUpdate(float dt) {
	//TODO
	//reset
	float aux = 0.083333333;
	float num = aux * mass * ((2 * 2) + (2 * 2));
	Ibody = {
		num,0,0,
		0,num,0,
		0,0,num
	};
	reset -= dt;
	if (reset <= 0) {
		PhysicsInit();
	}
	//calculs
	linearMomentum += dt*gravity;
	//angularMomentum += dt*torque;
	//printf("%f", );
	velocity = linearMomentum / mass;
	position += dt*velocity;
	mat3 algu = mat3_cast(quaternion);
	inertiaInvers = algu*inverse(Ibody)*transpose(algu);
	angularVelocity = inertiaInvers*angularMomentum;
	//quaternion = quat(0, angularVelocity);
	quaternion += (0.5f*quat(0, angularVelocity)*quaternion )*dt;
	quaternion = normalize(quaternion);
	//printf("%f", quaternion.y);
	rotatedMatrix = mat4_cast(quaternion);

	
	
	/*mat3 angularMat = {
		0,-angularVelocity.z,angularVelocity.y,
		angularVelocity.z,0,-angularVelocity.x,
		-angularVelocity.y,angularVelocity.x,0
	};
	angularMat = transpose(angularMat);*/
	
	//rotatedMatrix = mat3(rotatedMatrix) + dt*(angularMat*mat3(rotatedMatrix));
	
	mat4 tranlationMatrix = {
		1,0,0,0,
		0,1,0,0,
		0,0,1,0,
		position.x,position.y,position.z,1
	};
	//mat4 finalRotation = mat4(rotatedMatrix);
	mat4 finalMatrix = tranlationMatrix *rotatedMatrix;
	for (int i = 0; i < 7; i++)
	{
		cubeUpdateVertex[i] =finalMatrix*CubesPositionBuffer[i];
		if (cubeUpdateVertex[i].y <= 0) {

			collisionPlane(cubeUpdateVertex[i], inertiaInvers, vec3(0, 1, 0), velocity, angularVelocity,position);
		}
		else if (cubeUpdateVertex[i].y >= 10) {

			collisionPlane(cubeUpdateVertex[i], inertiaInvers, vec3(0, -1, 0), velocity, angularVelocity, position);
		}
		if (cubeUpdateVertex[i].x <= -5) {

			collisionPlane(cubeUpdateVertex[i], inertiaInvers, vec3(1, 0, 0), velocity, angularVelocity, position);
		}
		else if (cubeUpdateVertex[i].x >= 5) {

			collisionPlane(cubeUpdateVertex[i], inertiaInvers, vec3(-1, 0, 0), velocity, angularVelocity, position);
		}
		if (cubeUpdateVertex[i].z <= -5) {

			collisionPlane(cubeUpdateVertex[i], inertiaInvers, vec3(0, 0, 1), velocity, angularVelocity, position);
		}
		else if (cubeUpdateVertex[i].z >= 5) {

			collisionPlane(cubeUpdateVertex[i], inertiaInvers, vec3(0, 0, -1), velocity, angularVelocity, position);
		}
	}
	Cube::updateCube(finalMatrix);


}
void PhysicsCleanup() {
	//TODO
	Cube::cleanupCube();
}